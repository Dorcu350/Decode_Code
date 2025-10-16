package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp
public class TestShooter extends LinearOpMode {

    DcMotorEx motor_shooter;

    VoltageSensor sensor_volt;

    FtcDashboard dashboard;

    public static double Kp = 1.05, Ki = 0, Kd = 0, Kf,target_velo = 1500;
    PIDCoefficients coefficients = new PIDCoefficients(Kp,Ki,Kd);
    BasicPID controller = new BasicPID(coefficients);

    @Override
    public void runOpMode() throws InterruptedException {

        motor_shooter = hardwareMap.get(DcMotorEx.class, "motor");
        sensor_volt = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while(opModeIsActive()) {
            double ratio = 10 / sensor_volt.getVoltage();

            double curr_velo = motor_shooter.getVelocity();

            double power = controller.calculate(target_velo, curr_velo);

            power = Math.max(0.0, Math.min(1.0, power));

            if(gamepad1.dpad_up)
                motor_shooter.setPower(1 * ratio);

            if(gamepad1.dpad_down)
                motor_shooter.setPower(-1 * ratio);

            if(gamepad1.left_bumper)
                motor_shooter.setPower(0);

            if(gamepad1.right_bumper)
                motor_shooter.setPower(1);

            if(gamepad1.a)
                motor_shooter.setPower(power + Kf);


            telemetry.addData("current velocity: ", curr_velo);
            telemetry.addData("Error: ", target_velo - curr_velo);
            telemetry.addData("ce calc pid asta muie: ", power);
            telemetry.addData("target velocity: ", target_velo);
            telemetry.addData("current power: ", motor_shooter.getPower());
            telemetry.update();
        }
    }
}
