package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {

    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a)
                servo.setPosition(1);

            if(gamepad1.b)
                servo.setPosition(0);
        }
    }
}
