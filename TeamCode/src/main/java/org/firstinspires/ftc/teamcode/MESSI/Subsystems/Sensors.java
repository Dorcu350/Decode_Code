package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

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
    ColorSensor sensor_intake;
    DigitalChannel sensor_shooting, sensor_sorter, sensor_shooting_2;
    AnalogInput fourth_wire;
    Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;
    Globals globals;


    public boolean check_for_shooting() {return (!sensor_shooting.getState() || !sensor_shooting_2.getState()); }
    public boolean check_in_sorting() {return !sensor_sorter.getState();}
    public double readKickerPos() {return fourth_wire.getVoltage();}
    public boolean kickerRetracted() {return readKickerPos() >= 2.85;}
    public boolean kickerUp() {return readKickerPos() <= 2.47;}

    public double getTxAngle() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return Math.toRadians(result.getTx());
        }
        return 0;
    }

    public boolean onTarget() {
        return Math.abs(Globals.heading_error) <= 0.051;
    }

    public double getTa() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return Math.round(result.getTa() * 100) / 100.0;
        }
        return 0;
    }

    public void setGoalBlue() {
        limelight.pipelineSwitch(0);
    }

    public void setGoalRed() {
        limelight.pipelineSwitch(1);
    }

    public void setCheckMotif() {
        limelight.pipelineSwitch(2);
    }

    public boolean seeingTag() {
        LLResult result = limelight.getLatestResult();

        return result != null && result.isValid();
    }
    public boolean isGreen() {
        return showHSV() < 185.0 && showHSV() > 100.0;
    }
    public boolean isPurple() {
        return showHSV() < 240.0 && showHSV() > 195.0;
    }
    public double showHSV() {
        float[] hsvValues = new float[3];
        int red = sensor_intake.red();
        int blue = sensor_intake.blue();
        int green = sensor_intake.green();
        int max = Math.max(red, Math.max(green, blue));
        if(max == 0) max = 1;
        int normRed = (int) ((red / (float) max) * 255);
        int normBlue = (int) ((blue / (float) max) * 255);
        int normGreen = (int) ((green / (float) max) * 255);
        Color.RGBToHSV(normRed, normGreen, normBlue, hsvValues);
        return hsvValues[0];
    }
    public int showRed() {
        return sensor_intake.red();
    }

    public int showBlue() {
        return sensor_intake.blue();
    }

    public int showGreen() {
        return sensor_intake.green();
    }

//    public int showHue() {
//        return sensor_intake.argb();
//    }

    public Sensors(HardwareMap hardwareMap) {
        sensor_intake = hardwareMap.get(ColorSensor.class, "color");
//        sensor_shooting = hardwareMap.get(DigitalChannel.class, "feed");
//        sensor_shooting_2 = hardwareMap.get(DigitalChannel.class, "feed2");
//        sensor_sorter = hardwareMap.get(DigitalChannel.class, "sorter");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        fourth_wire = hardwareMap.get(AnalogInput.class, "4");

        limelight.setPollRateHz(100);   // Ask 100 times per second
        limelight.start();

        sensor_shooting.setMode(DigitalChannel.Mode.INPUT);
        sensor_shooting_2.setMode(DigitalChannel.Mode.INPUT);

        globals = new Globals();
    }
}
