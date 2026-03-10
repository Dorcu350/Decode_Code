package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@TeleOp
public class Test_4_DT extends LinearOpMode {

    DcMotorEx leftFront, rightFront,rightRear, leftRear,shooter,transfer,intake;
    Servo servo_drop;

    public static double s = 0.42;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        transfer = hardwareMap.get(DcMotorEx.class, "index");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        servo_drop = new CachingServo(hardwareMap.get(Servo.class, "drop"));


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.triangle) {
                rightRear.setPower(1);
                rightFront.setPower(1);
                leftFront.setPower(1);
                leftRear.setPower(1);
            }
            if(gamepad1.x)
                shooter.setPower(0.75);
            if(gamepad1.circle)
                intake.setPower(0.8);
            if (gamepad1.square)
                transfer.setPower(0.8);

            servo_drop.setPosition(s);


            telemetry.addData("leftFront ", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightRear ", rightRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftRear ", leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightFront ", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("shooter ", shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("intake ", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("transfer ", transfer.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
