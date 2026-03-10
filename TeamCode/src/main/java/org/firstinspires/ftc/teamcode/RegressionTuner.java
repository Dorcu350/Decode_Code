package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RITA.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.RITA.Utils.MultipleRegression;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "Regression Tuner - Real Time", group = "Tuning")
public class RegressionTuner extends LinearOpMode {

    DcMotorEx motor_shooter, motor_shooter_2;
    public CachingDcMotorEx motor_intake, motor_index;
    Servo servo_hood,servo_drop,servo_stopper;
    TelemetryManager telemetryM;

    // Flywheel PID/feedforward tunables
    public static double kP = 0.005, kS = 0.043, kV = 0.00035;

    // Example target (you can change which one you're aiming for)
    public static double TARGET_VELOCITY = 100; // default target in whatever units your getVelocity() uses
    public static double STOPPER_OPEN = 0.7, STOPPER_CLOSE = 0.45;
    Sensors sensors;
    Follower follower;

    MultipleRegression regression = new MultipleRegression();

    // ────────────────────────────────────────────────
    //  Dashboard-tunable regression points (single distance tuning)
    // ────────────────────────────────────────────────
    @Configurable
    public static class RegressionPoints {
        public static double testDistance = 135.0;

        // Point 1 (low rpm / higher hood usually)
        public static double rpm1 = 60;
        public static double angle1 = 0.42;

        // Point 2
        public static double rpm2 = 80;
        public static double angle2 = 0.35;

        // Point 3 (high rpm / flatter hood usually)
        public static double rpm3 = 100;
        public static double angle3 = 0.28;

        // Trigger to re-fit the regression
        public static boolean refitTrigger = false;

        // Just for visibility in dashboard
        public static int refitCount = 0;
    }

    // Track previous trigger state to detect changes
    private boolean lastRefitTrigger = false;
    private boolean gamepadYWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        motor_shooter   = hardwareMap.get(DcMotorEx.class, "shooter");
        motor_shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        servo_hood      = new CachingServo(hardwareMap.get(Servo.class, "hood"));
        motor_intake    = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"));
        motor_index     = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "index"));
        servo_drop = new CachingServo(hardwareMap.get(Servo.class, "drop"));
        servo_stopper      = new CachingServo(hardwareMap.get(Servo.class, "stopper"));

        sensors = new Sensors(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensors.pinpoint.setPosX(9, DistanceUnit.INCH);
        sensors.pinpoint.setPosY(8.5, DistanceUnit.INCH);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                sensors.pinpoint.getPosX(DistanceUnit.INCH),
                sensors.pinpoint.getPosY(DistanceUnit.INCH),
                sensors.pinpoint.getHeading(AngleUnit.RADIANS)
        ));
        follower.startTeleopDrive();

        // Initial regression setup (will be updated from Dashboard values)
        rebuildRegression();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            servo_drop.setPosition(0.42);
            servo_stopper.setPosition(STOPPER_CLOSE);

            // ────────────────────────────────────────────────
            //  Check for refit trigger (Dashboard or gamepad Y)
            // ────────────────────────────────────────────────
            boolean currentTrigger = RegressionPoints.refitTrigger;

            if (currentTrigger != lastRefitTrigger) {
                if (currentTrigger) {  // rising edge
                    rebuildRegression();
                    RegressionPoints.refitCount++;
                    telemetryM.addData("Regression refitted", RegressionPoints.refitCount + " times");
                }
                lastRefitTrigger = currentTrigger;
            }

            // Optional: gamepad Y to toggle refit (if you don't want to use phone)
            if (gamepad1.y && !gamepadYWasPressed) {
                RegressionPoints.refitTrigger = !RegressionPoints.refitTrigger;
                gamepadYWasPressed = true;
            }
            if (!gamepad1.y) gamepadYWasPressed = false;

            // ────────────────────────────────────────────────
            //  Shooter velocity control
            // ────────────────────────────────────────────────
            double currentVel = motor_shooter.getVelocity();  // IMPORTANT: know your units!
            // If your getVelocity() is in ticks/sec or rad/sec, convert if needed:
            // double currentRPM = currentVel * (60.0 / ticksPerRev / gearRatio); // example

            double targetVel = TARGET_VELOCITY;
            double power = kS + kV * targetVel + kP * (targetVel - currentVel);

            motor_shooter.setPower(power);
            motor_shooter_2.setPower(power);

            // ────────────────────────────────────────────────
            //  Hood command using current velocity
            // ────────────────────────────────────────────────
            double hoodPosition = regression.getHoodAngle(
                    RegressionPoints.testDistance,
                    currentVel
            );

            servo_hood.setPosition(hoodPosition);

            // ────────────────────────────────────────────────
            //  Intake/index test
            // ────────────────────────────────────────────────
            if (gamepad1.b) {
                motor_intake.setPower(-0.8);
                motor_index.setPower(0.8);
                servo_stopper.setPosition(STOPPER_OPEN);
            } else {
                servo_stopper.setPosition(STOPPER_CLOSE);
                motor_intake.setPower(0);
                motor_index.setPower(0);
            }

            // ────────────────────────────────────────────────
            //  Telemetry
            // ────────────────────────────────────────────────
            double distance = Math.hypot(144 - follower.getPose().getY(), 144 - follower.getPose().getX());

            telemetry.addData("Refit count", RegressionPoints.refitCount);
            telemetry.addData("Test Distance", "%.1f in", RegressionPoints.testDistance);
            telemetry.addData("Regression points", String.format(
                    "(%.0f → %.3f), (%.0f → %.3f), (%.0f → %.3f)",
                    RegressionPoints.rpm1, RegressionPoints.angle1,
                    RegressionPoints.rpm2, RegressionPoints.angle2,
                    RegressionPoints.rpm3, RegressionPoints.angle3
            ));
            telemetry.addData("Current velocity", "%.1f", currentVel);
            telemetry.addData("Target velocity", "%.1f", targetVel);
            telemetry.addData("Power", "%.4f", power);
            telemetry.addData("Hood position", "%.4f", hoodPosition);
            telemetry.addData("Robot → Goal distance", "%.1f in", distance);
            telemetry.update();

            follower.update();
        }
    }

    private void rebuildRegression() {
        regression = new MultipleRegression(); // reset

        regression.add(RegressionPoints.testDistance, RegressionPoints.rpm1, RegressionPoints.angle1);
        regression.add(RegressionPoints.testDistance, RegressionPoints.rpm2, RegressionPoints.angle2);
        regression.add(RegressionPoints.testDistance, RegressionPoints.rpm3, RegressionPoints.angle3);

        regression.create();
    }
}