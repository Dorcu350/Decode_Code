package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestSensor extends LinearOpMode {
    private DigitalChannel sensorOut;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorOut = hardwareMap.get(DigitalChannel.class, "s");  // Config name in config file
        sensorOut.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(opModeIsActive()) {
            boolean detected = !sensorOut.getState();  // Invert: low = detected (false → true)
            if (detected) {
                telemetry.addData("Status", "Object within 5cm!");
            } else {
                telemetry.addData("Status", "Path clear");
            }
            telemetry.update();
        }

    }
}
