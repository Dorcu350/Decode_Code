package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import static java.lang.Math.abs;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {
    ColorSensor sensor_intake;
    DigitalChannel sensor_shooting, sensor_sorter;
    Limelight3A limelight;

    public boolean check_for_shooting() {
        return !sensor_shooting.getState();
    }
    public boolean check_in_sorting() {return !sensor_sorter.getState();};

    public double getTx() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result.getTx() / 10;
        }
        return 0;
    }

    public boolean onTarget() {
        return getTx() < 1;
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
    public boolean tagInRange() {
        return getTa() <= 0.45 && getTa() >= 0.29;
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
        sensor_shooting = hardwareMap.get(DigitalChannel.class, "feed");
        sensor_sorter = hardwareMap.get(DigitalChannel.class, "sorter");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);   // Ask 100 times per second
        limelight.start();

        sensor_shooting.setMode(DigitalChannel.Mode.INPUT);
    }
}
