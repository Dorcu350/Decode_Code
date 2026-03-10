package org.firstinspires.ftc.teamcode.RITA.Subsystems;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RITA.Utils.Globals;

import java.util.concurrent.TimeUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class Intake {
    final TelemetryManager telemetryM;
    public CachingDcMotorEx motor_intake, motor_index;
    public CachingServo servo_shifter, servo_drop;
    public static double INTAKE_SHIFT_POS = 0.07, HANG_SHIFT_POS = 0.13, DROP_DOWN_POS = 0.42 , DROP_UP_POS = 0.39, RATIO_TRANSFER = 1, RATIO_INTAKE = 1, RATIO_SCALE;
    public static double kP = 0.1, kF = 0.1, nominal_voltage = 12.0;
    public static int TARGET_HANG = 800;
    VoltageSensor voltageSensor;
    Sensors sensors;
    InterpLUT transfer_speed = new InterpLUT();
    public enum State {
        INTAKE,
        STOP,
        FORCE_REVERSE,
        HANG,
        TRANSFER
    }
    public State state;
    public void update_intake(Gamepad gamepad) {
        if(Globals.force_drop)
            lower_intake();
        switch(state) {
            case STOP:
                shift_intake();
                if(!Globals.force_drop)
                    raise_intake();
                motor_intake.setPower(0);
                motor_index.setPower(0);

                if(Globals.start_transfer)
                    state = State.TRANSFER;

                break;

            case INTAKE:
                shift_intake();
                sensors.checkFullTransfer();
                if(!Globals.first_ball)
                    motor_index.setPower(1);
                else
                    motor_index.setPower(0);

                if(Globals.third_ball) {
                    gamepad.rumble(250);
                    motor_intake.setPower(0.3);
                    if(!Globals.force_drop)
                        raise_intake();
                } else {
                    lower_intake();
                    motor_intake.setPower(1);
                }

                if(Globals.start_transfer)
                    state = State.TRANSFER;


                break;

            case TRANSFER:
                shift_intake();
                lower_intake();
                if(Globals.distance_goal >= 60 && Globals.distance_goal <= 200) {
                    RATIO_SCALE = transfer_speed.get(Globals.distance_goal);
                    motor_intake.setPower(RATIO_SCALE);
                    motor_index.setPower(RATIO_SCALE);
                }

                if(!Globals.start_transfer)
                    state = State.INTAKE;

                break;

            case FORCE_REVERSE:
                shift_intake();
                lower_intake();
                motor_intake.setPower(-0.4);

                if(Globals.start_transfer)
                    state = State.TRANSFER;


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

        transfer_speed.add(65, 1);
        transfer_speed.add(115, 1);
        transfer_speed.add(135, 0.8);
        transfer_speed.add(200, 0.8);

        transfer_speed.createLUT();


        shift_intake();

        state = State.STOP;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
    }


}
