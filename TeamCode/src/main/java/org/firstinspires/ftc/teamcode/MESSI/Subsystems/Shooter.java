package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

@Configurable
public class Shooter {
    final TelemetryManager telemetryM;
    Sensors sensors;
    Globals globals;
    public boolean start_your_engines = false;
    public DcMotorEx motor_shooter, motor_shooter_2;
    public Servo servo_hood, servo_feeder, servo_block;
    public static double kP = 0.01, kI = 0, kD = 0, kF = 0.63;
    public static int target_velocity = 1300;
    public static double feeder_jos = 0.037, feeder_sus = 0.18, block_open = 0.7, block_close = 0.93;
    public static double hood_test = 0.84; //0.86 in spate de tot
    PIDCoefficients coef = new PIDCoefficients(kP, kI, kD);
    InterpLUT LUT = new InterpLUT();
    BasicPID pid = new BasicPID(coef);
    public enum State {
        IDLE,
        STOPPED,
        RUNNING,
        SHOOT
    }
    public State state;
    public static double deadBand = 0.01, lastValue;
    public void update_shooter() {
        double vel = motor_shooter.getVelocity();
        double pow;
        double error = abs(target_velocity - vel);

        servo_hood.setPosition(hood_test);

        if(start_your_engines) {
            pow = pid.calculate(target_velocity, vel);
            pow = max(0, Math.min(1, pow));
            motor_shooter.setPower(pow + kF);
            motor_shooter_2.setPower(pow + kF);
        }

        switch(state) {
            case SHOOT:
                servo_block.setPosition(block_open);
                start_your_engines = true;

//                if(sensors.seeingTag() && sensors.tagInRange()) {
//                    double raw = sensors.getTa();
//
//                    if(Math.abs(raw - lastValue) > deadBand) {
//                        lastValue = raw;
//                        servo_hood.setPosition(LUT.get(lastValue));
//                    }
//                }

                if(sensors.check_for_shooting() && error == 0 && sensors.onTarget())
                    servo_feeder.setPosition(feeder_sus);

                if(!sensors.check_for_shooting() && is_over_current())
                    servo_feeder.setPosition(feeder_jos);

                break;
            case STOPPED:
                start_your_engines = false;
                servo_feeder.setPosition(feeder_jos);
                servo_block.setPosition(block_close);
                motor_shooter_2.setPower(0);
                motor_shooter.setPower(0);
                break;
        }
    }
    public boolean is_over_current() {
        return motor_shooter.getCurrent(CurrentUnit.MILLIAMPS) > 3000;
    }
    public boolean is_spinning() {
        return motor_shooter.getPower() != 0;
    }
    public Shooter(HardwareMap hardwareMap) {
        motor_shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        motor_shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        servo_hood = hardwareMap.get(Servo.class, "hood");
        servo_feeder = hardwareMap.get(Servo.class, "feeder");
        servo_block = hardwareMap.get(Servo.class, "block");

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        servo_feeder.setPosition(feeder_jos);
        servo_hood.setPosition(hood_test);
        servo_block.setPosition(block_close);

        LUT.add(0.26, 0.8);
        LUT.add(0.29, 0.8);
        LUT.add(0.32, 0.79);
        LUT.add(0.38, 0.78);
        LUT.add(0.45, 0.76);
        LUT.add(0.47, 0.75);
        LUT.createLUT();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        globals = new Globals();
        sensors = new Sensors(hardwareMap);
        state = State.STOPPED;
    }
}
