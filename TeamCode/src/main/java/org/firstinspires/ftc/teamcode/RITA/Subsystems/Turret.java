package org.firstinspires.ftc.teamcode.RITA.Subsystems;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RITA.Utils.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
public class Turret {
    final TelemetryManager telemetryM;
    Sensors sensors;
    public CachingServo servo_left, servo_right;

    public static double position_test = 0.497, target_angle, relative_angle, target_position, error, position_hang = 0.1, offset = 0;
    public final double MIN_POS = 0.097, MAX_POS = 0.897, MIN_ANGLE = -120, MAX_ANGLE = 120;
    public static double X_GOAL_BLUE = 0, Y_GOAL_BLUE = 144, X_GOAL_RED = 144, Y_GOAL_RED = 144;
    public static double X_GOAL_BLUE_AUTO = 10, X_GOAL_RED_AUTO = 148;
    public static double shooterWorldX, shooterWorldY, shooterOffset = -1.377;

    public void update_turret(double x, double y, double heading) {
        shooterWorldX = x + (shooterOffset * Math.cos(heading));
        shooterWorldY = y + (shooterOffset * Math.sin(heading));

        if(Globals.faze == Globals.FAZE.TELEOP) {
            if (Globals.alliance == Globals.ALLIANCE.BLUE)
                target_angle = Math.atan2(Y_GOAL_BLUE - shooterWorldY, X_GOAL_BLUE - shooterWorldX);
            else
                target_angle = Math.atan2(Y_GOAL_RED - shooterWorldY, X_GOAL_RED - shooterWorldX);
        }else {
            if (Globals.alliance == Globals.ALLIANCE.BLUE)
                target_angle = Math.atan2(Y_GOAL_BLUE - shooterWorldY, X_GOAL_BLUE_AUTO - shooterWorldX);
            else
                target_angle = Math.atan2(Y_GOAL_RED - shooterWorldY, X_GOAL_RED_AUTO - shooterWorldX);
        }

        target_angle = AngleUnit.normalizeRadians(target_angle);
        heading = AngleUnit.normalizeRadians(heading);

        relative_angle = Math.toDegrees(target_angle - heading);
        relative_angle = AngleUnit.normalizeDegrees(relative_angle);

        relative_angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, relative_angle));
        target_position = Range.scale(relative_angle, MIN_ANGLE, MAX_ANGLE, MIN_POS, MAX_POS);

        if(!Globals.hanging)
            moveTo(target_position + offset);
        else
            moveTo(position_hang);
    }
    public void moveTo(double target) {
        servo_left.setPosition(target);
        servo_right.setPosition(target);
    }
    public double getPosition() {
        return servo_left.getPosition();
    }

    public Turret(HardwareMap hardwareMap) {
        servo_left  = new CachingServo(hardwareMap.get(Servo.class, "left_t"));
        servo_right = new CachingServo(hardwareMap.get(Servo.class, "right_t"));

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
    }
}