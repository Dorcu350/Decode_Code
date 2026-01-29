package org.firstinspires.ftc.teamcode.MESSI.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Hang {
    public CachingDcMotorEx motor_hang;
    public static int target, target_default = -50, target_engaged = 1600, target_defense = 35;
    public enum State {
        DEFENSE,
        DEFAULT,
        ENGAGED
    }
    public State state;
    public void update() {
        motor_hang.setTargetPosition(target);
        motor_hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_hang.setPower(1);

//        if(Math.abs(Globals.heading_error) >= 0.065 && Shooter.has_shot && !goHang)
//            state = State.DEFENSE;
//
//        if(!Shooter.has_shot && state != State.DEFAULT && !goHang)
//            state = State.DEFAULT;

        switch(state) {
            case DEFAULT:
                target = target_default;
                break;
            case ENGAGED:
                target = target_engaged;
                break;
            case DEFENSE:
                target = target_defense;
                break;
        }
    }

    public Hang(HardwareMap hardwareMap) {
        motor_hang = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "hang"));

        motor_hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_hang.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        target = target_default;
        motor_hang.setTargetPosition(target);
        motor_hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        state = State.DEFAULT;
    }
}
