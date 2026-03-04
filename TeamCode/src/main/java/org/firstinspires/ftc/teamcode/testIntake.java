package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testIntake extends LinearOpMode {
    Servo servo_intake;
    DcMotorEx motor_intake,motor_transfer;
    public static double pos_servo_intake = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo_intake = hardwareMap.get(Servo.class,"servo_intake");
        motor_intake = hardwareMap.get(DcMotorEx.class, "motor_intake");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "motor_transfer");

        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            servo_intake.setPosition(pos_servo_intake);

            if (gamepad1.circle) motor_intake.setPower(1);
            else motor_intake.setPower(0);

            if (gamepad1.triangle) motor_transfer.setPower(1);
            else motor_transfer.setPower(0);

            if (gamepad1.x) {
                motor_transfer.setPower(1);
                motor_intake.setPower(1);
            } else {
                motor_transfer.setPower(0);
                motor_intake.setPower(0);
            }
        }
    }
}
