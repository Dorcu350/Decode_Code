package org.firstinspires.ftc.teamcode.RITA.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public static double kP = 0.007, kS = 0.055, kV = 0.00034, nominal_voltage = 12.0;
    // public static double kP = 0.005, kS = 0.043, kV = 0.00035, nominal_voltage = 12.0;
    final double X_GOAL_BLUE = 0, Y_GOAL_BLUE = 144, X_GOAL_RED = 144, Y_GOAL_RED = 144;
    public static double TARGET_VELOCITY, ppr = 28, TARGET_TEST = 940, TARGET_STOPPED = 0, TARGET_LIMIT_LOWER = 150 , TARGET_LIMIT_UPPER = 100 ;
    public static double HOOD_TEST = 0.575 , HOOD_DESIRED, STOPPER_OPEN = 0.8, STOPPER_CLOSE = 0.48 ,error = TARGET_TEST; //0.45 in spate
    public double distance_from_goal;
    public static boolean initial_release = false, start_your_engines = false, auto = false;
    public static double offset = 0;
    public static double max_rpm;
    VoltageSensor voltageSensorShooter;
    public enum State {
        IDLE,
        STOPPED,
        RUNNING,
        SHOOT,
        FORCE_STOP
    }
    public State state;

    public void update_shooter(double x, double y, Gamepad gamepad) {
        double vel = motor_shooter.getVelocity();
        if(initial_release)
            error = abs(TARGET_VELOCITY - vel);

        if(start_your_engines) {
            double pow = ((kV * TARGET_VELOCITY) + (kP * (TARGET_VELOCITY - vel)) + kS);
            motor_shooter.setPower(pow);
            motor_shooter_2.setPower(pow);
        }

        if (Globals.alliance == Globals.ALLIANCE.BLUE)
            distance_from_goal = Math.hypot (Y_GOAL_BLUE - y, X_GOAL_BLUE - x);
        else {
            distance_from_goal = Math.hypot(Y_GOAL_RED - y, X_GOAL_RED - x);
        }

        Globals.distance_goal = distance_from_goal;


//        servo_hood.setPosition(HOOD_TEST);

        if(!Globals.hanging && distance_from_goal <= 170 && distance_from_goal >= 60) {
            TARGET_VELOCITY = calculateShooterRpm(distance_from_goal);
            servo_hood.setPosition(calculateOptimalAngle(distance_from_goal));
        }


        switch(state) {
            case IDLE:

                Globals.pre_spin = true;
                Globals.start_transfer = false;
                initial_release = true;
                start_your_engines = true;
                servo_stopper.setPosition(STOPPER_CLOSE);

//                if(distance_from_goal <= 195 && distance_from_goal >= 65)
//                    TARGET_VELOCITY = (vel_lut.get(distance_from_goal) - 140) + offset;

//                TARGET_VELOCITY = TARGET_TEST;


                break;

            case SHOOT:

                start_your_engines = true;
                Globals.pre_spin = false;

                gamepad.rumble(250);

//                if(distance_from_goal <= 195 && distance_from_goal >= 65)
//                    TARGET_VELOCITY = vel_lut.get(distance_from_goal) + offset;


//                TARGET_VELOCITY = TARGET_TEST;


                if(noError(error)) {
                    Globals.first_ball = false;
                    Globals.second_ball = false;
                    Globals.third_ball = false;
                    servo_stopper.setPosition(STOPPER_OPEN);
                    if(sensors.stopperOpen())
                        Globals.start_transfer = true;
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

                TARGET_VELOCITY = 100;

                Globals.start_transfer = false;
                Globals.pre_spin = false;
                start_your_engines = true;

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


//        vel_lut.add(65, 1120);
//        vel_lut.add(75, 1180);
//        vel_lut.add(85, 1220);
//        vel_lut.add(95, 1280);
//        vel_lut.add(105, 1360);
//        vel_lut.add(115, 1400);
//        vel_lut.add(125, 1700);
//        vel_lut.add(135, 1800);
//        vel_lut.add(145, 1900);
//        vel_lut.add(155, 2000);
//        vel_lut.add(165, 2100);
//        vel_lut.add(175, 2200);
//        vel_lut.add(185, 2320);
//        vel_lut.add(195, 2340);
//
//        vel_lut.createLUT();
//
//        regression.add(65, 1020, 0.35);
//        regression.add(65, 1120, 0.35);
//
//        regression.add(75, 1120, 0.35);
//        regression.add(75, 1180, 0.35);
//
//        regression.add(85, 1160, 0.37);
//        regression.add(85, 1220, 0.37);
//
//        regression.add(95, 1220, 0.37);
//        regression.add(95, 1280, 0.37);
//
//        regression.add(105, 1300, 0.4);
//        regression.add(105, 1360, 0.4);
//
//        regression.add(115, 1340, 0.4);
//        regression.add(115, 1400, 0.4);
//
//        regression.add(125, 1680, 0.475);
//        regression.add(125, 1700, 0.475);
//
//        regression.add(135, 1760, 0.475);
//        regression.add(135, 1780, 0.475);
//        regression.add(135, 1800, 0.475);
//
//        regression.add(145, 1860, 0.475);
//        regression.add(145, 1880, 0.475);
//        regression.add(145, 1900, 0.475);
//
//        regression.add(155, 1960, 0.475);
//        regression.add(155, 1980, 0.475);
//        regression.add(155, 2000, 0.475);
//
//        regression.add(165, 2060, 0.475);
//        regression.add(165, 2080, 0.475);
//        regression.add(165, 2100, 0.475);
//
//        regression.add(175, 2160, 0.475);
//        regression.add(175, 2180, 0.475);
//        regression.add(175, 2200, 0.475);
//
//        regression.add(185, 2280, 0.455);
//        regression.add(185, 2300, 0.465);
//        regression.add(185, 2320, 0.475);
//
//        regression.add(195, 2280, 0.475);
//        regression.add(195, 2300, 0.475);
//        regression.add(195, 2340, 0.475);
//
//        regression.create();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);

        state = State.IDLE;
        TARGET_VELOCITY = TARGET_STOPPED;
        servo_stopper.setPosition(STOPPER_CLOSE);
        voltageSensorShooter = hardwareMap.voltageSensor.iterator().next();

        telemetryM.addData("vel raw ", TARGET_VELOCITY - motor_shooter.getVelocity());
    }

    public double calculateShooterRpm(double distance) {
        return MathFunctions.clamp(-0.00045066 * Math.pow(distance, 3) + 0.154779 * Math.pow(distance, 2) - 11.19503 * distance + 1353.37995,  1100, 1800);
    }

    public double calculateOptimalAngle(double distance) {
        return MathFunctions.clamp((7.51101 * Math.pow(10, -8)) * Math.pow(distance, 3) - 0.0000334554 * Math.pow(distance, 2) + 0.00557613 * distance + 0.219282 , 0.45, 0.575);
    }
}
