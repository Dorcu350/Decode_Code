package org.firstinspires.ftc.teamcode.RITA.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.max;

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

import org.firstinspires.ftc.teamcode.RITA.Utils.Globals;
import org.firstinspires.ftc.teamcode.RITA.Utils.MultipleRegression;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class Shooter {
    final TelemetryManager telemetryM;
    Sensors sensors;
    public CachingDcMotorEx motor_shooter, motor_shooter_2;
    public CachingServo servo_hood, servo_stopper;
    public static double kP = 0.01, kS = 0.08, kV = 0.00036;
    final double X_GOAL_BLUE = 0, Y_GOAL_BLUE = 144, X_GOAL_RED = 144, Y_GOAL_RED = 144;
    public static double TARGET_VELOCITY, ppr = 28, TARGET_TEST = 940, TARGET_STOPPED = 0;
    public static double HOOD_TEST = 0.22 , HOOD_DESIRED, STOPPER_OPEN = 0.77, STOPPER_CLOSE = 0.9 ,error = TARGET_TEST; //0.32 in fata de tot
    public double distance_from_goal;
    public static boolean initial_release = false, start_your_engines = false, auto = false;
    public static double offset = 0;
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

        if(initial_release)
            error = abs(TARGET_VELOCITY - vel);

        if(start_your_engines) {
            motor_shooter.setPower((kV * TARGET_VELOCITY) + (kP * (TARGET_VELOCITY - vel)) + kS);
            motor_shooter_2.setPower((kV * TARGET_VELOCITY) + (kP * (TARGET_VELOCITY - vel)) + kS);
        }

        if (Globals.alliance == Globals.ALLIANCE.BLUE)
            distance_from_goal = Math.hypot(Y_GOAL_BLUE - y, X_GOAL_BLUE - x);
        else
            distance_from_goal = Math.hypot(Y_GOAL_RED - y, X_GOAL_RED - x);


//        if(distance_from_goal >= 135 && distance_from_goal <= 160) {
//            desired = regression_far.getHoodAngle(distance_from_goal, vel);
//            servo_hood.setPosition(HOOD_DESIRED);
//        }
//        else if(distance_from_goal <= 110 && distance_from_goal >= 60) {
//            desired = regression.getHoodAngle(distance_from_goal, vel);
//            servo_hood.setPosition(HOOD_DESIRED);
//        }else
        servo_hood.setPosition(HOOD_TEST);



        switch(state) {
            case IDLE:

                Globals.pre_spin = true;
                Globals.start_transfer = false;
                initial_release = true;
                start_your_engines = true;
                servo_stopper.setPosition(STOPPER_CLOSE);

//                if(distance_from_goal >= 135 && distance_from_goal <= 160)
//                    TARGET_VELOCITY = vel_lut_far.get(distance_from_goal) + offset;
//                else if(distance_from_goal <= 110 && distance_from_goal >= 60)
//                    TARGET_VELOCITY = vel_lut.get(distance_from_goal) + offset;
//                else
//                    TARGET_VELOCITY = prespin_velocity.getClosest(distance_from_goal);

                TARGET_VELOCITY = TARGET_TEST;

                break;

            case SHOOT:

                start_your_engines = true;
                Globals.start_transfer = true;
                Globals.pre_spin = false;

                gamepad.rumble(250);

//                if(distance_from_goal >= 135 && distance_from_goal <= 160)
//                    TARGET_VELOCITY = vel_lut_far.get(distance_from_goal) + offset;
//                else if(distance_from_goal <= 110 && distance_from_goal >= 60)
//                    TARGET_VELOCITY = vel_lut.get(distance_from_goal) + offset;
//                else
//                    TARGET_VELOCITY = prespin_velocity.getClosest(distance_from_goal);

                TARGET_VELOCITY = TARGET_TEST;

                if(noError(error)) {
                    Globals.first_ball = false;
                    Globals.second_ball = false;
                    Globals.third_ball = false;
                    servo_stopper.setPosition(STOPPER_OPEN);
                }

                break;

            case STOPPED:

                TARGET_VELOCITY = TARGET_STOPPED;

                Globals.start_transfer = false;
                Globals.pre_spin = false;
                start_your_engines = false;

                motor_shooter.setPower(0);
                motor_shooter_2.setPower(0);
                servo_stopper.setPosition(STOPPER_CLOSE);

                break;

            case FORCE_STOP:

                TARGET_VELOCITY = -1;

                Globals.start_transfer = false;
                Globals.pre_spin = false;
                start_your_engines = false;

                motor_shooter.setPower(-1);
                motor_shooter_2.setPower(-1);
                servo_stopper.setPosition(STOPPER_CLOSE);

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
        servo_hood      = new CachingServo(hardwareMap.get(Servo.class, "hood"));
        servo_stopper      = new CachingServo(hardwareMap.get(Servo.class, "stopper"));


        motor_shooter.  setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior    (DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter.  setMode                 (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode                 (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setDirection            (DcMotorSimple.Direction.REVERSE);


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
        TARGET_VELOCITY = TARGET_STOPPED;
        servo_stopper.setPosition(STOPPER_CLOSE);

        telemetryM.addData("vel raw ", TARGET_VELOCITY - motor_shooter.getVelocity());
    }
}
