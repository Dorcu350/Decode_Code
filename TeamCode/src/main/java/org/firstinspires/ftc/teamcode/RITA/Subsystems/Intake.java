package org.firstinspires.ftc.teamcode.RITA.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RITA.Utils.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class Intake {
    final TelemetryManager telemetryM;
    public CachingDcMotorEx motor_intake, motor_index;
    public CachingServo servo_shifter, servo_drop;
    public static double INTAKE_SHIFT_POS = 0.07, HANG_SHIFT_POS = 0.13, DROP_DOWN_POS = 0.6, DROP_UP_POS = 0.8;
    public static double kP = 0.1, kF = 0.1;
    public static int TARGET_HANG = 800;
    VoltageSensor voltageSensor;
    Sensors sensors;
    public enum State {
        INTAKE,
        STOP,
        FORCE_REVERSE,
        HANG,
        TRANSFER
    }
    public State state;
    public void update_intake(Gamepad gamepad) {
        if(Globals.start_transfer)
            state = State.TRANSFER;
        switch(state) {
            case STOP:

                shift_intake();
                raise_intake();
                motor_intake.setPower(0);
                motor_index.setPower(0);

                break;

            case INTAKE:

                shift_intake();
                sensors.checkFullTransfer();
                motor_intake.setPower(1);

                if(!Globals.first_ball)
                    motor_index.setPower(1);
                else
                    motor_index.setPower(0);

                if(Globals.third_ball) {
                    gamepad.rumble(250);
                    raise_intake();
                } else
                    lower_intake();

                break;

            case TRANSFER:

                shift_intake();
                lower_intake();
                motor_intake.setPower(1);
                motor_index.setPower(1);

                if(!Globals.start_transfer)
                    state = State.INTAKE;

                break;

            case FORCE_REVERSE:

                shift_intake();
                lower_intake();
                motor_intake.setPower(-1);

                break;

            case HANG:

                shift_hang();
                if(sensors.shifterInHang()) {
                    motor_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor_intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    double error = TARGET_HANG - motor_intake.getCurrentPosition();
                    motor_intake.setPower((kP * error ) + kF);
                }
                else
                    motor_intake.setPower(0.5);

                break;
        }
    }
    public void shift_hang() {
        servo_shifter.setPosition(HANG_SHIFT_POS);
    }
    public void shift_intake() {
        servo_shifter.setPosition(INTAKE_SHIFT_POS);
    }
    public void lower_intake() {
        servo_drop.setPosition(DROP_DOWN_POS);
    }
    public void raise_intake() {
        servo_drop.setPosition(DROP_UP_POS);
    }
    public boolean is_intaking() {
        return motor_intake.getPower() != 0;
    }
    public double get_current() {return motor_intake.getCurrent(CurrentUnit.MILLIAMPS);}
    public boolean is_over_current() {return get_current() > 2500;}
    public Intake(HardwareMap hardwareMap) {
        motor_intake  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"));
        servo_shifter = new CachingServo(hardwareMap.get(Servo.class, "shifter"));
        servo_drop = new CachingServo(hardwareMap.get(Servo.class, "drop"));
        motor_index     = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "index"));

        motor_intake.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.BRAKE);
        motor_intake.setDirection            (DcMotorSimple.Direction.REVERSE);
        motor_intake.setCachingTolerance     (0.05);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shift_intake();

        state = State.STOP;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
    }
}
