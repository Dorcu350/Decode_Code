package org.firstinspires.ftc.teamcode.StopC.Subsyst;

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

import org.firstinspires.ftc.teamcode.StopC.Utils.Globals;
import org.firstinspires.ftc.teamcode.StopC.Utils.MultipleRegression;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class Shooter {

    final TelemetryManager telemetryM;
    Sensors sensors;
    public CachingDcMotorEx motor_shooter, motor_shooter_2, motor_index;
    public CachingServo servo_hood;
    public static double kP = 0.03, kI = 0, kD = 0, kS = 0.055, kV = 0.00035, kA = 0;
    final double x_goal_blue = 0, y_goal_blue = 144, x_goal_red = 144, y_goal_red = 144;
    public static double target_velocity, ppr = 28, target_far = 1020, target_close = 940, target_stopped = 0;
    public static double block_open = 0.72, block_close = 0.69;
    public static double main_jos = 0.58, main_sus = 0.76;
//     public static double main_jos = 0.13, main_sus = 0.19, main_full = 0.24;
    public static double hood_test = 0.11, error = target_far , hood_close = 0.65 ; //067 in spate de tot
    public double distance_from_goal;
    public static boolean initial_release = false, start_your_engines = false, auto = false;
    PIDCoefficients coef = new PIDCoefficients(kP, kI, kD);
    FeedforwardCoefficients coef_ff = new FeedforwardCoefficients(kV, kA, kS);
    public static double hoodDeadband = 0.01, lastHoodPosition = 0, desired;
    BasicPID pid = new BasicPID(coef);
    public ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    BasicFeedforward feedforward = new BasicFeedforward(coef_ff);
    public enum State {
        IDLE,
        STOPPED,
        RUNNING,
        SHOOT
    }
    public State state;
    MultipleRegression regression = new MultipleRegression();
    MultipleRegression regression_far = new MultipleRegression();
    InterpLUT vel_lut = new InterpLUT();
    InterpLUT vel_lut_far = new InterpLUT();


    public void update_shooter(double x, double y) {
        double vel = motor_shooter.getVelocity();
        double fb, ff;

        if(initial_release)
            error = abs(target_velocity - vel);

        if(start_your_engines) {
            fb = pid.calculate(target_velocity, vel);
            ff = feedforward.calculate(vel, target_velocity, 0);

            ff = max(0, Math.min(1, ff));
            fb = max(0, Math.min(1, fb));

            motor_shooter.setPower(fb + ff);
            motor_shooter_2.setPower(fb + ff);
        }

        if (Globals.alliance == Globals.ALLIANCE.BLUE)
            distance_from_goal = Math.hypot(y_goal_blue - y, x_goal_blue - x);
        else
            distance_from_goal = Math.hypot(y_goal_red - y, x_goal_red - x);


        if(distance_from_goal >= 135 && distance_from_goal <= 160)
            desired = regression_far.getHoodAngle(distance_from_goal, vel);
        else if(distance_from_goal <= 100 && distance_from_goal >= 60)
            desired = regression.getHoodAngle(distance_from_goal, vel);

        servo_hood.setPosition(desired);

        switch(state) {
            case IDLE:
                Globals.pre_spin = true;
                initial_release = true;
                start_your_engines = true;


                if(distance_from_goal >= 135 && distance_from_goal <= 160)
                    target_velocity = vel_lut_far.get(distance_from_goal);
                else if(distance_from_goal <= 100 && distance_from_goal >= 60)
                    target_velocity = vel_lut.get(distance_from_goal);
                else
                    target_velocity = 600;

                break;
            case SHOOT:
                start_your_engines = true;
                Globals.start_feeding = true;
                Globals.pre_spin = false;

                if(distance_from_goal >= 135 && distance_from_goal <= 160)
                    target_velocity = vel_lut_far.get(distance_from_goal);
                else if(distance_from_goal <= 100 && distance_from_goal >= 60)
                    target_velocity = vel_lut.get(distance_from_goal);
                else
                    target_velocity = 600;

                if(noError(error))
                    motor_index.setPower(1);

                break;
            case STOPPED:
                target_velocity = target_stopped;

                Globals.start_feeding = false;
                Globals.pre_spin = false;
                start_your_engines = false;

                motor_shooter.setPower(0);
                motor_shooter_2.setPower(0);
                motor_index.setPower(0);
                break;
        }
    }
    public boolean is_spinning() {
        return (target_velocity == target_far || target_velocity == target_close) && state == State.RUNNING;
    }

    public double getRpm(double velocity) {
        return ( (double) velocity / ppr ) * 60;
    } // aici aveti cu getRPM daca va trb

    public boolean noError(double error) {
        return error <= 20;
    }

    public double getVelocity(double rpm) {
        return (double) (rpm * ppr) / 60;
    }

    public Shooter(HardwareMap hardwareMap) {
        motor_shooter   = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter"));
        motor_shooter_2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter2"));
        motor_index     = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "index"));

        servo_hood          = new CachingServo(hardwareMap.get(Servo.class, "hood"));

        motor_shooter.  setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter.  setMode                 (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode                 (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setDirection            (DcMotorSimple.Direction.REVERSE);
        motor_index.    setDirection            (DcMotorSimple.Direction.REVERSE);


        motor_shooter.setCachingTolerance(0.001);
        motor_shooter_2.setCachingTolerance(0.001);

        timer.startTime();

        vel_lut.add(60, 740);
        vel_lut.add(70, 760);
        vel_lut.add(80, 780);
        vel_lut.add(90, 820);
        vel_lut.add(100, 860);

        vel_lut.createLUT();

        regression.add(60, 700, 0.11);
        regression.add(60, 740, 0.12);

        regression.add(70, 720, 0.11);
        regression.add(70, 760, 0.12);

        regression.add(80, 740, 0.11);
        regression.add(80, 780, 0.12);

        regression.add(90, 780, 0.11);
        regression.add(90, 820, 0.12);

        regression.add(100, 820, 0.11);
        regression.add(100, 860, 0.12);

        regression.create();

//

        regression_far.add(135, 960, 0.13);
        regression_far.add(135, 1040, 0.14);

        regression_far.add(140, 1000, 0.13);
        regression_far.add(140, 1040, 0.14);

        regression_far.add(145, 1020, 0.13);
        regression_far.add(145, 1060, 0.14);

        regression_far.add(150, 1040, 0.13);
        regression_far.add(150, 1080, 0.14);

        regression_far.add(155, 1080, 0.13);
        regression_far.add(155, 1120, 0.14);

        regression_far.add(160, 1080, 0.13);
        regression_far.add(160, 1120, 0.14);

        regression_far.create();


        vel_lut_far.add(135, 1040);
        vel_lut_far.add(140, 1040);
        vel_lut_far.add(145, 1060);
        vel_lut_far.add(150, 1080);
        vel_lut_far.add(155, 1120);
        vel_lut_far.add(160, 1120);

        vel_lut_far.createLUT();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
        state = State.STOPPED;
        target_velocity = target_stopped;
    }
}
