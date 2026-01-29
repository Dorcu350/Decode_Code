package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.StopC.Subsyst.Shooter;

@TeleOp
public class FeedingTest extends LinearOpMode {
    Servo kicker;

    @Override
    public void runOpMode() throws InterruptedException {

        kicker = hardwareMap.get(Servo.class, "main_k");

        waitForStart();

        while (opModeIsActive()) {
            kicker.setPosition(Shooter.main_jos);
        }
    }
}
