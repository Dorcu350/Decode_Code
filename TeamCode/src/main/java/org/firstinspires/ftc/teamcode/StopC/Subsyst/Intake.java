package org.firstinspires.ftc.teamcode.StopC.Subsyst;

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
import org.firstinspires.ftc.teamcode.StopC.Utils.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class Intake {
    final TelemetryManager telemetryM;
    public CachingDcMotorEx motor_intake;
    public CachingServo servo_shifter;
    public static double shifter_intake = 0.075  , shifter_hang;
    VoltageSensor voltageSensor;
    Sensors sensors;
    public enum State {
        INTAKE,
        REVERSE,
        STOP,
        FORCE_REVERSE
    }
    public State state;
    public void update_intake(Gamepad gamepad) {
        double ratio = 12 / voltageSensor.getVoltage();
        servo_shifter.setPosition(shifter_intake);

        switch(state) {
            case STOP:
                motor_intake.setPower(0);

                if(Globals.start_feeding)
                    state = State.INTAKE;
                break;
            case INTAKE:
                motor_intake.setPower(1 * ratio);

                if(is_over_current())
                    gamepad.rumble(250);
                break;
            case FORCE_REVERSE:
                motor_intake.setPower(-1);
                break;
        }
    }
    public boolean is_intaking() {
        return motor_intake.getPower() != 0;
    }
    public double get_current() {return motor_intake.getCurrent(CurrentUnit.MILLIAMPS);}
    public boolean is_over_current() {return get_current() > 2500;}
    public Intake(HardwareMap hardwareMap) {
        motor_intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"));
        servo_shifter = new CachingServo(hardwareMap.get(Servo.class, "shifter"));

        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_intake.setCachingTolerance(0.05);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        servo_shifter.setPosition(shifter_intake);

        state = State.STOP;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
    }
}
