package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sensors {
    ColorSensor sensor_intake;
    DigitalChannel sensor_shooting;
    Limelight3A limelight;

    public boolean check_for_shooting() {
        return !sensor_shooting.getState();
    }

    public double getTx() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result.getTx() / 10;
        }
        return 0;
    }

    public boolean isGreen() {
        if(showBlue() > showRed() && showBlue() - 5 > showGreen())
            return false;
        return showGreen() > showRed() && showGreen() + 5 > showBlue();
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

    public int showHue() {
        return sensor_intake.argb();
    }

    public Sensors(HardwareMap hardwareMap) {
        sensor_intake = hardwareMap.get(ColorSensor.class, "color");
        sensor_shooting = hardwareMap.get(DigitalChannel.class, "feed");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);   // Ask 100 times per second
        limelight.pipelineSwitch(0);    // Make sure pipeline 0 is AprilTag
        limelight.start();

        sensor_shooting.setMode(DigitalChannel.Mode.INPUT);
    }
}
