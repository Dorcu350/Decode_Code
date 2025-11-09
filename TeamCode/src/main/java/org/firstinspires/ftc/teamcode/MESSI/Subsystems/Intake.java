package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

@Configurable
public class Intake {
    final TelemetryManager telemetryM;
    DcMotorEx motor_intake;
    Sorter sorter;
    Servo servo_block;
    Sensors sensors;
    Globals globals;
    Shooter shooter;
    public enum State {
        INTAKE,
        INTAKE_WHILE_SORTING,
        STOP
    }
    public State state;
    int detected_color = -1;
    public static double push_ball = 0.85, let_ball = 0;
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public boolean firstBall = false, secondBall = false, hasGreen = false, inSorter = false, eject = false, aDropat = false, daPush = false;
    public void update_intake(Gamepad gamepad) {

        if(daPush)
            pushBall();
        else
            letBall();

        switch(state) {
            case STOP:
                motor_intake.setPower(0);
                sorter.resetIndexer();
                if(shooter.is_spinning() && !sensors.check_for_shooting()) {
                    state = State.INTAKE;
                    timer.reset();
                }
                break;
            case INTAKE:
                motor_intake.setPower(1);
                if(!sensors.check_for_shooting() && timer.milliseconds() > 700 && timer.milliseconds() < 750)
                    daPush = true;
                if(timer.milliseconds() > 1100)
                    daPush = false;
                if(timer.milliseconds() > 4000)
                    timer.reset();

                if(is_over_current())
                    gamepad.rumble(250);

                if(sensors.check_for_shooting() && shooter.is_spinning())
                    state = State.STOP;

                break;
            case INTAKE_WHILE_SORTING:
                daPush = true;
                if(!eject)
                    motor_intake.setPower(0.8);
                else if(sensors.check_in_sorting())
                    motor_intake.setPower(-1);
                else if(!sensors.check_in_sorting() && !is_over_current()) {
                    eject = false;
                    aDropat = true;
                }

                if(is_over_current()) {
                    if (sensors.isGreen()) detected_color = 0;
                    else if (sensors.isPurple()) detected_color = 1;
                    else detected_color = -1;
                }

                switch (globals.motif) {
                    case PPG:
                        if (!firstBall) {
                            if (detected_color == 0) {
                                sorter.openLatch();
                                sorter.sendUp();
                            }

                            if(!inSorter && sensors.check_for_shooting())
                                firstBall = true;

                            if(sensors.check_in_sorting() && !inSorter) {
                                sorter.resetIndexer();
                                sorter.closeLatch();
                                inSorter = true;
                                firstBall = true;
                            }

                        } else if (!secondBall) {
                            if (detected_color == 0 && !inSorter)
                                sorter.sendUp();

                            if(inSorter && sensors.check_for_shooting())
                                secondBall = true;

                            else if(!inSorter && sensors.check_in_sorting()) {
                                sorter.resetIndexer();
                                sorter.closeLatch();
                                inSorter = true;
                                secondBall = true;
                            }
                        }
                        break;

                    case PGP:
                        if (!firstBall) {
                            if (detected_color == 0) {
                                sorter.openLatch();
                                sorter.sendUp();
                                hasGreen = true;
                            }

                            if(!inSorter && sensors.check_for_shooting())
                                firstBall = true;

                            if(sensors.check_in_sorting() && !inSorter) {
                                sorter.resetIndexer();
                                sorter.closeLatch();
                                inSorter = true;
                                firstBall = true;
                            }

                        } else if (!secondBall) {
                            if (detected_color == 1 && !inSorter)
                                sorter.sendUp();

                            if(inSorter && sensors.check_for_shooting()) {
                                secondBall = true;
                                if(hasGreen) {
                                    sorter.drop();
                                    sorter.openLatch();
                                    eject = true;
                                }
                            }

                            else if(!inSorter && sensors.check_in_sorting()) {
                                sorter.resetIndexer();
                                sorter.closeLatch();
                                inSorter = true;
                                secondBall = true;
                            }
                        }
                        break;

                    case GPP:
                        if (!firstBall) {
                            if (detected_color == 1) {
                                sorter.openLatch();
                                sorter.sendUp();
                            }

                            if(!inSorter && sensors.check_for_shooting())
                                firstBall = true;

                            if(sensors.check_in_sorting() && !inSorter) {
                                sorter.resetIndexer();
                                sorter.closeLatch();
                                inSorter = true;
                                firstBall = true;
                            }

                        } else if (!secondBall) {
                            if (detected_color == 1 && inSorter) {
                                sorter.openLatch();
                                sorter.sendUp();
                            }

                            if(inSorter && sensors.check_for_shooting())
                                secondBall = true;

                            else if(!inSorter && sensors.check_in_sorting()) {
                                sorter.resetIndexer();
                                sorter.closeLatch();
                                inSorter = true;
                                secondBall = true;
                            }
                        }
                        break;
                }

                break;
        }
    }
    public void pushBall() {
        servo_block.setPosition(push_ball);
    }
    public void letBall() {
        servo_block.setPosition(let_ball);
    }
    public boolean is_intaking() {
        return motor_intake.getPower() != 0;
    }
    public double get_current() {return motor_intake.getCurrent(CurrentUnit.MILLIAMPS);}
    public boolean is_over_current() {return get_current() > 1000;};

    public Intake(HardwareMap hardwareMap) {
        motor_intake = hardwareMap.get(DcMotorEx.class, "intake");
        servo_block = hardwareMap.get(Servo.class, "drop");

        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_intake.setDirection(DcMotorSimple.Direction.REVERSE);

        state = State.STOP;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        sorter = new Sorter(hardwareMap);
        globals = new Globals();
        shooter = new Shooter(hardwareMap);
        sensors = new Sensors(hardwareMap);
        timer.startTime();
    }
}
