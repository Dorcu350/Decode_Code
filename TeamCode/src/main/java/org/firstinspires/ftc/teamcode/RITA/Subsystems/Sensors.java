package org.firstinspires.ftc.teamcode.RITA.Subsystems;

import static java.lang.Math.abs;

import android.graphics.Color;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RITA.Utils.Globals;

public class Sensors {
    DigitalChannel sensor_third, sensor_second, sensor_first;
    AnalogInput shifter_a;
    Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;
    Globals globals;


     //Transfer
    public boolean readFirst() {
        return (!sensor_first.getState());
    }

    public boolean readSecond() {
         return (!sensor_second.getState());
    }

    public boolean readThird() {
        return (!sensor_third.getState());
    }

    public void checkFullTransfer() {
        if(!sensor_first.getState())
            Globals.first_ball = true;

        if(Globals.first_ball && !sensor_second.getState())
            Globals.second_ball = true;

        if(Globals.second_ball && !sensor_third.getState())
            Globals.third_ball = true;
    }

    //Shifter

    public double readShifterAnalog() {return shifter_a.getVoltage();};
    public boolean shifterInHang() {return readShifterAnalog() <= 2.8;};
    public boolean shifterInIntake() {return readShifterAnalog() >= 2.9;};

    //Turret

//    public double readTurretAnalog() {return turret_a.getVoltage();};


    public Sensors(HardwareMap hardwareMap) {
        sensor_first      = hardwareMap.get(DigitalChannel.class, "feed1");
        sensor_second     = hardwareMap.get(DigitalChannel.class, "feed2");
        sensor_third      = hardwareMap.get(DigitalChannel.class, "feed3");
        pinpoint          = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        shifter_a         = hardwareMap.get(AnalogInput.class, "shifter_a");
//        limelight         = hardwareMap.get(Limelight3A.class, "limelight");


        pinpoint.setOffsets(-185.55388, -78.40065, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        sensor_first.  setMode(DigitalChannel.Mode.INPUT);
        sensor_second. setMode(DigitalChannel.Mode.INPUT);
        sensor_third.  setMode(DigitalChannel.Mode.INPUT);

    }
}
