package org.firstinspires.ftc.teamcode.StopC.Subsyst;

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

import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

public class Sensors {
    DigitalChannel sensor_shooting, sensor_shooting_2;
    AnalogInput main_kicker_wire;
    Limelight3A limelight;
//    Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;
    Globals globals;


    //Shooter
    public boolean checkForShooting() {return (!sensor_shooting.getState() || !sensor_shooting_2.getState()); }

    //Feeder
    public double readMainKicker() {return main_kicker_wire.getVoltage();}
    public boolean mainKickerRetracted() {return readMainKicker() <= 0.45;}
    public boolean mainKickerUp() {return readMainKicker() >= 0.64;} //Apr\ ox

    //Camera
//x

    public boolean onTarget() {
        return Math.abs(Globals.heading_error) <= 0.051;
    }

//    public double getTa() {
////        LLResult result = limelight.getLatestResult();
//
//        if (result != null && result.isValid()) {
//            return Math.round(result.getTa() * 100) / 100.0;
//        }
//        return 0;
//    }

//    public void setGoalBlue() {
//        limelight.pipelineSwitch(0);
//    }
//
//    public void setGoalRed() {
//        limelight.pipelineSwitch(1);
//    }
//
//    public void setCheckMotif() {
//        limelight.pipelineSwitch(2);
//    }
//
//    public boolean seeingTag() {
//        LLResult result = limelight.getLatestResult();
//
//        return result != null && result.isValid();
//    }

    public Sensors(HardwareMap hardwareMap) {
        sensor_shooting = hardwareMap.get(DigitalChannel.class, "feed");
        sensor_shooting_2 = hardwareMap.get(DigitalChannel.class, "feed2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        main_kicker_wire = hardwareMap.get(AnalogInput.class, "main_w");

//        limelight.setPollRateHz(100);   // 100 / s
//        limelight.start();

        sensor_shooting.setMode(DigitalChannel.Mode.INPUT);
        sensor_shooting_2.setMode(DigitalChannel.Mode.INPUT);


        globals = new Globals();
    }
}
