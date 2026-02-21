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
import com.qualcomm.robotcore.hardware.Gamepad;
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
    public static double kP = 0.01, kI = 0, kD = 0, kS = 0.065, kV = 0.00036, kA = 0;
    final double x_goal_blue = 0, y_goal_blue = 139.5, x_goal_red = 139.5, y_goal_red = 139.5;
    public static double target_velocity, ppr = 28, target_prespin = 940, target_stopped = 0;
    public static double hood_test = 0.34, error = target_prespin; //0.34 in spate de tot
    public double distance_from_goal;
    public static boolean initial_release = false, start_your_engines = false, auto = false;
    public static double desired, offset = 0;
    LUT<Double, Integer> prespin_velocity = new LUT<Double, Integer>()
    {{
       add(85.0, 900);
       add(130.0, 1300);
    }};
    public enum State {
        IDLE,
        STOPPED,
        RUNNING,
        SHOOT,
        FORCE_STOP
    }
    public State state;
    MultipleRegression regression = new MultipleRegression();
    MultipleRegression regression_far = new MultipleRegression();
    InterpLUT vel_lut = new InterpLUT();
    InterpLUT vel_lut_far = new InterpLUT();

    public void update_shooter(double x, double y, Gamepad gamepad) {
        double vel = motor_shooter.getVelocity();
//        double fb, ff;

        if(initial_release)
            error = abs(target_velocity - vel);

        if(start_your_engines) {
            motor_shooter.setPower((kV * target_velocity) + (kP * (target_velocity - vel)) + kS);
            motor_shooter_2.setPower((kV * target_velocity) + (kP * (target_velocity - vel)) + kS);
        }

        if (Globals.alliance == Globals.ALLIANCE.BLUE)
            distance_from_goal = Math.hypot(y_goal_blue - y, x_goal_blue - x);
        else
            distance_from_goal = Math.hypot(y_goal_red - y, x_goal_red - x);


        if(distance_from_goal >= 135 && distance_from_goal <= 160) {
            desired = regression_far.getHoodAngle(distance_from_goal, vel);
            servo_hood.setPosition(desired);
        }
        else if(distance_from_goal <= 110 && distance_from_goal >= 60) {
            desired = regression.getHoodAngle(distance_from_goal, vel);
            servo_hood.setPosition(desired);
        }else
            servo_hood.setPosition(hood_test);



        switch(state) {
            case IDLE:
                Globals.pre_spin = true;
                Globals.start_feeding = false;
                initial_release = true;
                start_your_engines = true;
                motor_index.setPower(0);

                if(distance_from_goal >= 135 && distance_from_goal <= 160)
                    target_velocity = vel_lut_far.get(distance_from_goal) + offset;
                else if(distance_from_goal <= 110 && distance_from_goal >= 60)
                    target_velocity = vel_lut.get(distance_from_goal) + offset;
                else
                   target_velocity = prespin_velocity.getClosest(distance_from_goal);

//                target_velocity = target_prespin;


                break;
            case SHOOT:
                start_your_engines = true;
                Globals.start_feeding = true;
                Globals.pre_spin = false;

                gamepad.rumble(250);

                if(distance_from_goal >= 135 && distance_from_goal <= 160)
                    target_velocity = vel_lut_far.get(distance_from_goal) + offset;
                else if(distance_from_goal <= 110 && distance_from_goal >= 60)
                    target_velocity = vel_lut.get(distance_from_goal) + offset;
               else
                  target_velocity = prespin_velocity.getClosest(distance_from_goal);

//                target_velocity = target_prespin;

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
            case FORCE_STOP:
                target_velocity = -1;

                Globals.start_feeding = false;
                Globals.pre_spin = false;
                start_your_engines = false;

                motor_shooter.setPower(-1);
                motor_shooter_2.setPower(-1);
                motor_index.setPower(0);
                break;
        }
    }
    public boolean is_spinning() {
        return state == State.RUNNING;
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
        servo_hood      = new CachingServo(hardwareMap.get(Servo.class, "hood"));

        motor_shooter.  setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter.  setMode                 (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode                 (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setDirection            (DcMotorSimple.Direction.REVERSE);
        motor_index.    setDirection            (DcMotorSimple.Direction.REVERSE);


        motor_shooter.  setCachingTolerance(0.001);
        motor_shooter_2.setCachingTolerance(0.001);


        vel_lut.add(60, 940);
        vel_lut.add(70, 980);
        vel_lut.add(80, 1000);
        vel_lut.add(90, 1040);
        vel_lut.add(100, 1100);
        vel_lut.add(110, 1140);

        vel_lut.createLUT();

        regression.add(60, 940, 0.34);
        regression.add(60, 960, 0.34);

        regression.add(70, 960, 0.33);
        regression.add(70, 980, 0.33);

        regression.add(80, 980, 0.32);
        regression.add(80, 1000, 0.32);

        regression.add(90, 1020, 0.32);
        regression.add(90, 1040, 0.32);

        regression.add(100, 1080, 0.31);
        regression.add(100, 1100, 0.31);

        regression.add(110, 1120, 0.31);
        regression.add(110, 1140, 0.31);

        regression.create();

//

//        regression_far.add(135, 920, 0.08);
        regression_far.add(135, 1200, 0.275);
        regression_far.add(135, 1320, 0.275);

//        regression_far.add(137.5, 960, 0.08);
        regression_far.add(137.5, 1200, 0.275);
        regression_far.add(137.5, 1320, 0.275);

//        regression_far.add(140, 940, 0.085);
        regression_far.add(140, 1220, 0.275);
        regression_far.add(140, 1340, 0.275);

//        regression_far.add(142.5, 940, 0.085);
        regression_far.add(142.5, 1220, 0.275);
        regression_far.add(142.5, 1340, 0.275);

//        regression_far.add(145, 960, 0.085);
        regression_far.add(145, 1240, 0.275);
        regression_far.add(145, 1360, 0.275);

//        regression_far.add(147.5, 960, 0.085);
        regression_far.add(147.5, 1240, 0.275);
        regression_far.add(147.5, 1360, 0.275);

//        regression_far.add(150, 980, 0.085);
        regression_far.add(150, 1260, 0.275);
        regression_far.add(150, 1380, 0.275);

//        regression_far.add(152.5, 980, 0.085);
        regression_far.add(152.5, 1260, 0.275);
        regression_far.add(152.5, 1380, 0.275);

//        regression_far.add(155, 1000, 0.085);
        regression_far.add(155, 1300, 0.275);
        regression_far.add(155, 1400, 0.275);

//        regression_far.add(157.5, 1000, 0.085);
        regression_far.add(157.5, 1280, 0.275);
        regression_far.add(157.5, 1400, 0.275);

//        regression_far.add(160, 1020, 0.085);
        regression_far.add(160, 1300, 0.275);
        regression_far.add(160, 1420, 0.275);

        regression_far.create();


        vel_lut_far.add(135, 1320);
        vel_lut_far.add(137.5, 1320);
        vel_lut_far.add(140, 1340);
        vel_lut_far.add(142.5, 1340);
        vel_lut_far.add(145, 1360);
        vel_lut_far.add(147.5, 1360);
        vel_lut_far.add(150, 1380);
        vel_lut_far.add(152.5, 1380);
        vel_lut_far.add(155, 1400);
        vel_lut_far.add(157.5, 1400);
        vel_lut_far.add(160, 1420);

        vel_lut_far.createLUT();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
        state = State.IDLE;
        target_velocity = target_stopped;
        telemetryM.addData("vel raw ", target_velocity - motor_shooter.getVelocity());
    }
}
