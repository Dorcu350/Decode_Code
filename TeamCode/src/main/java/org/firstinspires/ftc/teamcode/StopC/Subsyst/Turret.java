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
    // 0.69 180 grade
    // 0.43 0 grade


    //-137.53 , 153
    final TelemetryManager telemetryM;
    Sensors sensors;
    public CachingServo servo_left, servo_right;
    public static double position_test = 0.505, target_angle, relative_angle, target_position, error, auto_blue = 0.4, auto_red = 0.33, decalation = 0, coef = 0.00344;
    public final double min_poz = 0.918, max_poz = 0.092, min_angle = -120, max_angle = 120;
    final double x_goal_blue = 0, y_goal_blue = 144, x_goal_red = 144, y_goal_red = 144;
    public void update_turret(double x, double y, double heading) {
        if(Globals.alliance == Globals.ALLIANCE.BLUE) {
            target_angle = Math.atan2(y_goal_blue - y, x_goal_blue - x);
        }
        else {
            target_angle = Math.atan2(y_goal_red - y, x_goal_red - x);
        }
        target_angle = AngleUnit.normalizeRadians(target_angle);
        heading =  AngleUnit.normalizeRadians(heading);

        relative_angle = Math.toDegrees(target_angle - heading);
        relative_angle = AngleUnit.normalizeDegrees(relative_angle);

        relative_angle = Math.max(min_angle, Math.min(max_angle, relative_angle));
        target_position = Range.scale(relative_angle, min_angle, max_angle, min_poz, max_poz);

        if(Globals.faze == Globals.FAZE.TELEOP)
            moveTo(target_position);
//        if(Globals.faze != Globals.FAZE.TELEOP && Globals.alliance == Globals.ALLIANCE.BLUE)
//            moveTo(auto_blue);
//        else if(Globals.faze != Globals.FAZE.TELEOP)
//         moveTo(auto_red);

//        moveTo(position_test);
    }
    public void moveTo(double target) {
        servo_left.setPosition(target);
        servo_right.setPosition(target);
    }

    public double getPosition() {
        return servo_left.getPosition();
    }
    public Turret(HardwareMap hardwareMap) {
        servo_left= new CachingServo(hardwareMap.get(Servo.class, "left_t"));
        servo_right= new CachingServo(hardwareMap.get(Servo.class, "right_t"));

        servo_right.setDirection(Servo.Direction.REVERSE);

//        if(Globals.alliance == Globals.ALLIANCE.BLUE)
//            moveTo(auto_blue);
//        else
//            moveTo(auto_red);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sensors = new Sensors(hardwareMap);
    }
}
