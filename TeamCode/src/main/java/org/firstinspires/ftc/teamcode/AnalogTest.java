package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class AnalogTest extends LinearOpMode {
    AnalogInput fourth;
    @Override
    public void runOpMode() throws InterruptedException {
        fourth = hardwareMap.get(AnalogInput.class, "fourth");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("voltage double ", fourth.getVoltage());
            telemetry.addData("voltage max ", fourth.getMaxVoltage());
            telemetry.update();
        }
    }
}
