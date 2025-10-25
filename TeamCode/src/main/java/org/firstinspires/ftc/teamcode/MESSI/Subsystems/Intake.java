package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

public class Intake {
    DcMotorEx motor_intake;
    Sorter sorter;
    Sensors sensors;
    Globals globals;
    public enum State {
        INTAKE,
        INTAKE_WHILE_SORTING,
        STOP
    }
    public State state;
    public int k = 0;
    boolean prevBallAtDestination = false;
    int detected_color = -1;
    boolean atDestination, keep_sending = true;
    public void update_intake() {
        switch(state) {
            case STOP:
                motor_intake.setPower(0);
                sorter.resetIndexer();
                prevBallAtDestination = atDestination;
                if(k == 0)
                    keep_sending = true;
                break;
            case INTAKE:
                motor_intake.setPower(1);
                break;
            case INTAKE_WHILE_SORTING:
                motor_intake.setPower(0.8);

                detected_color = -1;
                if (is_over_current() && is_intaking()) {
                    if (sensors.isGreen()) detected_color = 0;
                    else if (sensors.isPurple()) detected_color = 1;
                }
//                switch (globals.motif) {
//                    case PPG:
//                        if(detected_color == )
//                        break;
//                    case GPP:
//                        break;
//                    case PGP:
//                        break;
//
//                }
//                detected_color = -1;
//                if (is_over_current() && is_intaking()) {
//                    if (sensors.isGreen()) detected_color = 0;
//                    else if (sensors.isPurple()) detected_color = 1;
//
//                    if (detected_color != -1) {
//                        sorter.current_order[k] = detected_color;
//
//                        if (detected_color != sorter.target_order[k] && keep_sending) {
//                            sorter.sendUp();
//                            keep_sending = false;
//                        }
//                    }
//                }
//
//                atDestination = false;
//                if (sorter.current_order[k] != -1) {
//                    if (sorter.current_order[k] != sorter.target_order[k]) {
//                        atDestination = sensors.check_in_sorting();
//                    } else {
//                        atDestination = sensors.check_for_shooting();
//                    }
//                }
//
//                if (atDestination && !prevBallAtDestination) {
//                    k = (k + 1) % 3;
//                    state = State.STOP;
//                }
                break;
        }
    }

    public boolean is_intaking() {
        return motor_intake.getPower() != 0;
    }
    public double get_current() {return motor_intake.getCurrent(CurrentUnit.MILLIAMPS);}
    public boolean is_over_current() {return get_current() > 800;};

    public Intake(HardwareMap hardwareMap) {
        motor_intake = hardwareMap.get(DcMotorEx.class, "intake");

        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        state = State.STOP;

        sorter = new Sorter(hardwareMap);
        globals = new Globals();
        sensors = new Sensors(hardwareMap);
    }
}
