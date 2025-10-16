package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor motor_intake;
    public enum State {
        INTAKE,
        INTAKE_WHILE_SORTING,
        STOP
    }
    public State state;
    public void update_intake() {
        switch(state) {
            case STOP:
                motor_intake.setPower(0);
                break;
            case INTAKE:
                motor_intake.setPower(1);
                break;
            case INTAKE_WHILE_SORTING:
                break;
        }
    }

    public boolean is_intaking() {
        return motor_intake.getPower() != 0;
    }

    public Intake(HardwareMap hardwareMap) {
        motor_intake = hardwareMap.get(DcMotor.class, "intake");

        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        state = State.STOP;
    }
}
