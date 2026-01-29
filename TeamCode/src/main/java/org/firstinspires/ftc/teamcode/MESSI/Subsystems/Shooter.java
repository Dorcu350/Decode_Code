package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
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

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

//@Configurable
public class Shooter {
    final TelemetryManager telemetryM;
    Sensors sensors;
    Globals globals;
    public CachingDcMotorEx motor_shooter, motor_shooter_2;
    public CachingServo servo_block, servo_hood, servo_feeder;
    public static double kP = 0.02, kI = 0, kD = 0, kS = 0.055, kV = 0.0003, kA = 0;
    public static int target_velocity, ppr = 28, target_far = 1020, target_close_close = 840, target_close = 860, target_stopped = 0;
    public static double feeder_jos = 0.07, feeder_sus = 0.23, block_open = 0.4, block_close = 0.6;
    public static double hood_test = 0.67, error = target_far , hood_close = 0.67; //067 in spate de tot
    public static boolean initial_release = false, start_feeding = false, has_shot = false, start_your_engines = false, is_idle = false, auto = false, check_align = true;
    PIDCoefficients coef = new PIDCoefficients(kP, kI, kD);
    FeedforwardCoefficients coef_ff = new FeedforwardCoefficients(kV, kA, kS);
    InterpLUT LUT_Far = new InterpLUT();
    public LUT<Double, Integer> zone = new LUT<Double, Integer>() {{
        add(0.2, target_far);
        add(0.3, target_far);
        add(0.4, target_far);
        add(0.8, target_close);
        add(1.2, target_close_close);
        add(1.5, target_close_close);
    }};
    BasicPID pid = new BasicPID(coef);
    BasicFeedforward feedforward = new BasicFeedforward(coef_ff);
    public enum State {
        IDLE,
        STOPPED,
        RUNNING,
        SHOOT
    }
    public State state;
    public static double deadBand = 0.03, lastValue;
    public void update_shooter() {
        double vel = motor_shooter.getVelocity();
        double fb, ff;

        if(initial_release)
            error = abs(target_velocity - vel);

//        servo_hood.setPosition(hood_test);

        if(start_your_engines) {
            fb = pid.calculate(target_velocity, vel);
            ff = feedforward.calculate(vel, target_velocity, 0);

            ff = max(0, Math.min(1, ff));
            fb = max(0, Math.min(1, fb));

            motor_shooter.setPower(fb + ff);
            motor_shooter_2.setPower(fb + ff);
        }

        switch(state) {
            case IDLE:

                is_idle = true;
                initial_release = true;
                start_your_engines = true;
                if(error < 200 && sensors.check_for_shooting())
                    letBall();

                if(!auto)
                    target_velocity = zone.getClosest(sensors.getTa());
                else
                    target_velocity = target_close_close;

                break;

            case SHOOT:

//                if(!auto)
                    target_velocity = zone.getClosest(sensors.getTa());
//                else
//                    target_velocity = target_close_close;

                start_feeding = true;
                start_your_engines = true;
                is_idle = false;

                if(error < 200)
                    letBall();

                if(sensors.seeingTag()) {
                    double raw = sensors.getTa();

                    if(Math.abs(raw - lastValue) > deadBand) {
                        lastValue = raw;
                        if(target_velocity == target_far)
                            servo_hood.setPosition(LUT_Far.get(sensors.getTa()));
                        else if(target_velocity == target_close || target_velocity == target_close_close)
                            servo_hood.setPosition(hood_close);
                    }
                }

                if(noError(error) && sensors.check_for_shooting()) {
                    if(!check_align)
                        servo_feeder.setPosition(feeder_sus);
                    else if(sensors.onTarget())
                        servo_feeder.setPosition(feeder_sus);
                }


                if(sensors.kickerUp()) {
                    servo_feeder.setPosition(feeder_jos);
                    has_shot = true;
                }

                break;

            case STOPPED:

                target_velocity = target_stopped;

                start_feeding = false;
                is_idle = false;
                has_shot = false;
                start_your_engines = false;

                motor_shooter.setPower(0);
                motor_shooter_2.setPower(0);

                servo_feeder.setPosition(feeder_jos);
                servo_block.setPosition(block_close);

                break;
        }
    }
    public boolean is_over_current() {
        return motor_shooter.getCurrent(CurrentUnit.MILLIAMPS) > 4000;
    }
    public boolean is_spinning() {
        return (target_velocity == target_far || target_velocity == target_close) && state == State.RUNNING;
    }

    public double getRpm(double velocity) {
        return ( (double) velocity / ppr ) * 60;
    }

    public boolean noError(double error) {
        return error <= 20;
    }

    public double getVelocity(double rpm) {
        return (double) (rpm * ppr) / 60;
    }
    public void letBall() {
        servo_block.setPosition(block_open);
    }
    public void goDownKickerAAA() {
        servo_feeder.setPosition(feeder_jos);
    }
    public void shooterSpeedFailsafe(int add) {
        target_velocity = target_velocity + add;
    }
    public Shooter(HardwareMap hardwareMap) {
        motor_shooter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter"));
        motor_shooter_2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter2"));

        servo_hood = new CachingServo(hardwareMap.get(Servo.class, "hood"));
        servo_feeder = new CachingServo(hardwareMap.get(Servo.class, "feeder"));
        servo_block = new CachingServo(hardwareMap.get(Servo.class, "block"));

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_shooter.setCachingTolerance(0.001);
        motor_shooter_2.setCachingTolerance(0.001);

        servo_feeder.setPosition(feeder_jos);
        servo_hood.setPosition(hood_test);
        servo_block.setPosition(block_close);


//        LUT_Close.add(0.79, 0.62);
//        LUT_Close.add(0.8, 0.62);
//        LUT_Close.add(1.15, 0.63);
//        LUT_Close.add(1.29, 0.67);
//        LUT_Close.add(1.51, 0.67);
//        LUT_Close.add(2, 0.67);
//        LUT_Close.add(2.5, 0.67);
//        LUT_Close.add(3, 0.67);
//        LUT_Close.createLUT();

        LUT_Far.add(0.28, 0.67);
        LUT_Far.add(0.30, 0.67);
        LUT_Far.add(0.31, 0.67);
        LUT_Far.add(0.34, 0.65);
        LUT_Far.add(0.38, 0.63);
        LUT_Far.add(0.40, 0.62);
        LUT_Far.add(0.44, 0.59);
        LUT_Far.add(0.45, 0.59);
        LUT_Far.add(0.47, 0.59);
        LUT_Far.createLUT();

//        LUT_Far.add(0.28, 0.67);
//        LUT_Far.add(0.30, 0.67);
//        LUT_Far.add(0.31, 0.67);
//        LUT_Far.add(0.34, 0.65);
//        LUT_Far.add(0.39, 1060);
//        LUT_Far.add(0.38, 1060);
//        LUT_Far.add(0.44, 1020);
//        LUT_Far.add(0.45, 1020);
//        LUT_Far.add(0.47, 1000);
//        LUT_Far.createLUT();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        globals = new Globals();
        sensors = new Sensors(hardwareMap);
        state = State.STOPPED;
        target_velocity = target_stopped;
    }
}
