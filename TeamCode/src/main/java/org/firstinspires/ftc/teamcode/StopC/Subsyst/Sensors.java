//package org.firstinspires.ftc.teamcode.StopC.Subsyst;
//
//import static java.lang.Math.abs;
//
//import android.graphics.Color;
//
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;
//
//public class Sensors {
//    DigitalChannel sensor_shooting, sensor_shooting_2;
//    AnalogInput shifter_a, turret_a;
//    Limelight3A limelight;
//    public GoBildaPinpointDriver pinpoint;
//    Globals globals;
//
//
//    //Shooter
////    public boolean checkForShooting() {return (!sensor_shooting.getState() || !sensor_shooting_2.getState()); }
//
//    //Shifter
//
//    public double readShifterAnalog() {return shifter_a.getVoltage();};
//    public boolean shifterInHang() {return readShifterAnalog() <= 2.8;};
//    public boolean shifterInIntake() {return readShifterAnalog() >= 2.9;};
//
//    //Turret
//
//    public double readTurretAnalog() {return turret_a.getVoltage();};
//
//
//    public Sensors(HardwareMap hardwareMap) {
////        sensor_shooting   = hardwareMap.get(DigitalChannel.class, "feed");
////        sensor_shooting_2 = hardwareMap.get(DigitalChannel.class, "feed2");
////        limelight         = hardwareMap.get(Limelight3A.class, "limelight");
//        pinpoint          = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        shifter_a         = hardwareMap.get(AnalogInput.class, "shifter_a");
//        turret_a          = hardwareMap.get(AnalogInput.class, "turret_a");
//
//        pinpoint.setOffsets(-185.55388, -78.40065, DistanceUnit.MM);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
////        sensor_shooting.  setMode   (DigitalChannel.Mode.INPUT);
////        sensor_shooting_2.setMode   (DigitalChannel.Mode.INPUT);
//
//        globals = new Globals();
//    }
//}
