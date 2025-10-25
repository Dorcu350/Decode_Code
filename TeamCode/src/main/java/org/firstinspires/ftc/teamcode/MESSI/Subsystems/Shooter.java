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

import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

@Configurable
public class Shooter {
    final TelemetryManager telemetryM;
    Sensors sensors;
    Globals globals;
    public boolean start_your_engines = false;
    public DcMotorEx motor_shooter, motor_shooter_2;
    public Servo servo_hood, servo_feeder;
    public static double kP = 1.05, kI = 0, kD = 0, kF = 0.01;
    public static int target_velocity = 1500;
    public static double feeder_jos = 0.05, feeder_sus = 0.17;
    public static double hood_test = 0.92; //0.92 in spate de tot
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

        if(sensors.seeingTag()) {
            double raw = sensors.getTa();

            if(Math.abs(raw - lastValue) > deadBand) {
                lastValue = raw;
                servo_hood.setPosition(LUT.get(lastValue));
            }
        }

//        servo_hood.setPosition(hood_test);
//
//        servo_hood.setPosition(hood_test);

        if(start_your_engines) {
            pow = pid.calculate(target_velocity, vel);
            pow = max(0, Math.min(1, pow));
            motor_shooter.setPower(pow + kF);
            motor_shooter_2.setPower(pow + kF);
        }

        switch(state) {
            case RUNNING:
                start_your_engines = true;
                if(!sensors.check_for_shooting())
                    servo_feeder.setPosition(feeder_jos);
                break;
            case SHOOT:
                start_your_engines = true;
                if(sensors.check_for_shooting() && error <= 20) {
                    servo_feeder.setPosition(feeder_sus);
                    state = State.IDLE;
                }
                break;
            case IDLE:
                start_your_engines = true;
                if(!sensors.check_for_shooting())
                    servo_feeder.setPosition(feeder_jos);
                if(error <= 40)
                    state = State.RUNNING;
                break;
            case STOPPED:
                start_your_engines = false;
                motor_shooter_2.setPower(0);
                motor_shooter.setPower(0);
                break;
        }
    }

    public boolean is_spinning() {
        return motor_shooter.getPower() == 0;
    }
    public Shooter(HardwareMap hardwareMap) {
        motor_shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        motor_shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        servo_hood = hardwareMap.get(Servo.class, "hood");
        servo_feeder = hardwareMap.get(Servo.class, "feeder");

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo_feeder.setPosition(feeder_jos);
        servo_hood.setPosition(hood_test);

        LUT.add(0.27, 0.915);
        LUT.add(0.3, 0.915);
        LUT.add(0.35, 0.89);
        LUT.add(0.36, 0.88);
        LUT.add(0.43, 0.87);
        LUT.add(0.48, 0.865);
//        LUT.add(0.37, 0.85);
        LUT.createLUT();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        globals = new Globals();
        sensors = new Sensors(hardwareMap);
        state = State.STOPPED;
    }
}
