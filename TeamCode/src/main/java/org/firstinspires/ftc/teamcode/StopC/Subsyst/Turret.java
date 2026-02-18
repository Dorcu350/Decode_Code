package org.firstinspires.ftc.teamcode.StopC.Subsyst;

import static com.pedropathing.math.MathFunctions.normalizeAngle;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.StopC.Utils.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import static java.lang.Math.*;

@Configurable
public class Turret {
    final TelemetryManager telemetryM;
    Sensors sensors;
    public CachingServo servo_left, servo_right;
    public static double position_test = 0.505, target_angle, relative_angle, target_position, error, position_hang = 0.93;
    public final double min_poz = 0.9183, max_poz = 0.0916, min_angle = -120, max_angle = 120;
    public static double x_goal_blue = 0, y_goal_blue = 144, x_goal_red = 144, y_goal_red = 144;
    public static double x_goal_blue_auto = 10, x_goal_red_auto = 134;

    public void update_turret(double x, double y, double heading) {
        if(Globals.faze == Globals.FAZE.TELEOP) {
            if (Globals.alliance == Globals.ALLIANCE.BLUE)
                target_angle = Math.atan2(y_goal_blue - y, x_goal_blue - x);
            else
                target_angle = Math.atan2(y_goal_red - y, x_goal_red - x);
        }else {
            if (Globals.alliance == Globals.ALLIANCE.BLUE)
                target_angle = Math.atan2(y_goal_blue - y, x_goal_blue_auto - x);
            else
                target_angle = Math.atan2(y_goal_red - y, x_goal_red_auto - x);
        }

        target_angle = AngleUnit.normalizeRadians(target_angle);
        heading =  AngleUnit.normalizeRadians(heading);

        relative_angle = Math.toDegrees(target_angle - heading);
        relative_angle = AngleUnit.normalizeDegrees(relative_angle);

        relative_angle = Math.max(min_angle, Math.min(max_angle, relative_angle));
        target_position = Range.scale(relative_angle, min_angle, max_angle, min_poz, max_poz);

        if(!Globals.hanging)
            moveTo(target_position);
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

        servo_right.setDirection(Servo.Direction.REVERSE);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
    }
}
