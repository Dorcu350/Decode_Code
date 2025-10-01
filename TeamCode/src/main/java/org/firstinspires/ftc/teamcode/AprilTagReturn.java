package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class AprilTagReturn extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);   // Ask 100 times per second
        limelight.pipelineSwitch(0);    // Make sure pipeline 0 is AprilTag
        limelight.start();              // Start camera

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                    double tx = result.getTx(); // Horizontal offset
                    double ty = result.getTy(); // Vertical offset
                    double ta = result.getTa(); // Target area (%)

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }
}
