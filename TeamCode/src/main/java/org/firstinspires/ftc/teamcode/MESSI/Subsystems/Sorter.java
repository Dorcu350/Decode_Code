package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

//@Configurable
public class Sorter {
    final TelemetryManager telemetryM;
    CachingServo servo_sort, servo_latch;
    Globals globals;
    Sensors sensors;

    // 0.4 sort upwards 1
    public static double sort_upwards = 0.4, sort_return = 0, drop_upwards = 0.5, latch_open =  0.37, latch_close = 0.5, latch_teleOP = 0.67;
    //INT 0 = G si 1 = P
    int[] target_order = new int[3];
    public void setTarget() {
        switch (globals.motif) {
            case PPG:
                target_order[0] = 1;
                target_order[1] = 1;
                target_order[2] = 0;
                break;
            case GPP:
                target_order[0] = 0;
                target_order[1] = 1;
                target_order[2] = 1;
                break;
            case PGP:
                target_order[0] = 1;
                target_order[1] = 0;
                target_order[2] = 1;
                break;
        }
    }
    public void sendUp() {
        servo_sort.setPosition(sort_upwards);
    }
    public void resetIndexer() {
        servo_sort.setPosition(sort_return);
    }
    public void drop() {
        servo_sort.setPosition(drop_upwards);
    }
    public void closeLatch() {
        servo_latch.setPosition(latch_close);
    }
    public void openLatch() {
        servo_latch.setPosition(latch_open);
    }

    public Sorter(HardwareMap hardwareMap) {
        servo_sort = new CachingServo(hardwareMap.get(Servo.class, "sort"));
        servo_latch = new CachingServo(hardwareMap.get(Servo.class, "latch"));

        servo_sort.setPosition(sort_return);
        servo_latch.setPosition(latch_teleOP);

        globals = new Globals();
        sensors = new Sensors(hardwareMap);
        setTarget();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }
}
