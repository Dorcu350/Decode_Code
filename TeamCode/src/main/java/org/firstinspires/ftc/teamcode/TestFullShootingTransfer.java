package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@TeleOp
public class TestFullShootingTransfer extends LinearOpMode {

    DcMotorEx motor_shooter, motor_shooter_2;
    Servo servo_hood;
    public CachingDcMotorEx motor_intake, motor_index;
    TelemetryManager telemetryM;
    VoltageSensor sensor_volt;

    FtcDashboard dashboard;

    public static double power_tuned = 0, ratio;

    @Override
    public void runOpMode() throws InterruptedException {

        motor_shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        motor_shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        motor_intake  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"));
        motor_index     = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "index"));
        servo_hood = hardwareMap.get(Servo.class, "hood");

        sensor_volt = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                motor_shooter.setPower(power_tuned);
                motor_shooter.setPower(power_tuned);
            }
            if(gamepad1.b) {
                motor_intake.setPower(-ratio);
                motor_index.setPower(ratio);
            }
        }
    }
}
