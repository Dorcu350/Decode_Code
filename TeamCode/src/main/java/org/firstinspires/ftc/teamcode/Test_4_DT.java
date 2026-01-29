package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Test_4_DT extends LinearOpMode {

    DcMotor leftFront, rightFront,rightRear, leftRear;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a)
                leftFront.setPower(1);
            if(gamepad1.b)
                leftRear.setPower(1);
            if(gamepad1.x)
                rightFront.setPower(1);
            if(gamepad1.y)
                rightRear.setPower(1);
        }
    }
}
