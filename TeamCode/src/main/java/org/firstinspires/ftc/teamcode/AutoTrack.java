package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class AutoTrack extends LinearOpMode {
    Limelight3A limelight;
    DcMotor motor;

    double full_rot = 141.1, one_degree = 0.403;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight.setPollRateHz(100);   // Ask 100 times per second
        limelight.pipelineSwitch(0);    // Make sure pipeline 0 is AprilTag
        limelight.start();              // Start camera


        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                motor.setTargetPosition(33);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx(); // Horizontal offset
                double ty = result.getTy(); // Vertical offset
                double ta = result.getTa(); // Target area (%)

                if(gamepad1.b) {
                    motor.setTargetPosition(33 + (int)tx);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(1);
                }

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }

    }
    public double toTick(double x) {
        return x * one_degree;
    }

    public double toDegree(double x) {
        return x / one_degree;
    }
}
