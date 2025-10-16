package org.firstinspires.ftc.teamcode.MESSI;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.MESSI.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;
import org.firstinspires.ftc.teamcode.MESSI.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@TeleOp
@Configurable
public class Teleuaper extends LinearOpMode {
    public static Pose startingPose = new Pose(0, 0, 0);
    Follower follower;
    Shooter shooter;
    Intake intake;
    Sensors sensors;
    ElapsedTime timer;
    Globals globals;;
    boolean autoLock = true;
    double targetHeading;
    PIDFController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        globals = new Globals();
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive();
        follower.update();

        controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        timer.startTime();

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        while(opModeIsActive()) {
            shooter.update_shooter();
            intake.update_intake();
            follower.update();

            targetHeading = (follower.getHeading() - sensors.getTx());
            double error = targetHeading - follower.getHeading();
            controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
            controller.updateError(error);

            if(!autoLock) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.1,
                        (gamepad1.left_trigger - gamepad1.right_trigger),
                        true // Robot Centric
                );
            }else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * 0.4,
                        -gamepad1.left_stick_x * 0.4,
                        controller.run(),
                        true // Robot Centric
                );
            }

            if(gamepad1.a)
                autoLock = false;
            if(gamepad1.b)
                autoLock = true;

            if(gamepad1.dpad_up)
                shooter.servo_feeder.setPosition(0);

            if(gamepad1.right_bumper && timer.milliseconds() > 250) {
                if(!shooter.is_spinning())
                    shooter.state = Shooter.State.IDLE;
                if(shooter.state == Shooter.State.RUNNING)
                    shooter.state = Shooter.State.SHOOT;
                timer.reset();
            }

            if(gamepad1.left_bumper) {
                if (intake.is_intaking() && timer.milliseconds() > 250) {
                    intake.state = Intake.State.STOP;
                    timer.reset();
                } else if (!intake.is_intaking() && !globals.sorter_active && timer.milliseconds() > 250) {
                    intake.state = Intake.State.INTAKE;
                    timer.reset();
                } else if (!intake.is_intaking() && globals.sorter_active && timer.milliseconds() > 250) {
                    intake.state = Intake.State.INTAKE_WHILE_SORTING;
                    timer.reset();
                }
            }

            telemetry.addData("flywheel state running ", shooter.state == Shooter.State.RUNNING);
            telemetry.addData("flywheel state shoot ", shooter.state == Shooter.State.SHOOT);
            telemetry.addData("flywheel state idle ", shooter.state == Shooter.State.IDLE);
            telemetry.addData("sensor feed ", sensors.check_for_shooting());
            telemetry.addData("vel ", shooter.motor_shooter.getVelocity());
            telemetry.addData("heading ", follower.getHeading());
            telemetry.addData("tx ", sensors.getTx());
            telemetry.addData("red ", sensors.showRed());
            telemetry.addData("blue " ,sensors.showBlue());
            telemetry.addData("green ", sensors.showGreen());
            telemetry.addData("este verde ", sensors.isGreen());
            telemetry.addData("hue ", sensors.showHue());
            telemetry.update();
        }
    }
}
