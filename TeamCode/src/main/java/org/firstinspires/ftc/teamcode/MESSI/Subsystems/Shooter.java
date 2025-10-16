package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import static java.lang.Math.abs;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
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
    public DcMotorEx motor_shooter, motor_shooter_2;
    public Servo servo_hood, servo_feeder;
    public static double kP = 1.05, kI = 0, kD = 0, kF = 0.05;
    public static int target_velocity = 1500;
    public static double feeder_jos = 0.06, feeder_sus = 0.17;
    PIDCoefficients coef = new PIDCoefficients(kP, kI, kD);
    BasicPID pid = new BasicPID(coef);
    public enum State {
        IDLE,
        STOPPED,
        RUNNING,
        SHOOT
    }
    public State state;
    public void update_shooter() {
        double vel = motor_shooter.getVelocity();
        double pow;

        if(globals.start_flywheel) {
            pow = pid.calculate(target_velocity, vel);
            motor_shooter.setPower(pow + kF);
            motor_shooter_2.setPower(pow + kF);
        }

        switch(state) {
            case RUNNING:
                if(!sensors.check_for_shooting())
                    servo_feeder.setPosition(feeder_jos);
                break;
            case SHOOT:
                if(sensors.check_for_shooting()) {
                    servo_feeder.setPosition(feeder_sus);
                    state = State.IDLE;
                }
                break;
            case IDLE:
                if(!sensors.check_for_shooting())
                    servo_feeder.setPosition(feeder_jos);
                if(abs(target_velocity - vel) <= 40)
                    state = State.RUNNING;
                break;
            case STOPPED:
                motor_shooter.setPower(0);
                motor_shooter_2.setPower(0);
                if(globals.start_flywheel)
                    state = State.IDLE;
                break;
        }
    }

    public boolean is_spinning() {
        return motor_shooter.getPower() != 0;
    }
    public Shooter(HardwareMap hardwareMap) {
        motor_shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        motor_shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        servo_hood = hardwareMap.get(Servo.class, "hood");
        servo_feeder = hardwareMap.get(Servo.class, "feeder");

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo_feeder.setPosition(feeder_jos);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        globals = new Globals();
        sensors = new Sensors(hardwareMap);
        state = State.STOPPED;
    }
}
