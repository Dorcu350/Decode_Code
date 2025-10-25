package org.firstinspires.ftc.teamcode.MESSI.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MESSI.Utils.Globals;

@Configurable
public class Sorter {
    final TelemetryManager telemetryM;
    Servo servo_sort, servo_drop;
    Globals globals;
    public static double sort_upwards = 0.4, sort_return = 0, drop_upwards = 0.5, drop_return = 1;
    //INT 0 = G si 1 = P
    int[] target_order = new int[3];
    int[] current_order = new int[3];
    public Sorter(HardwareMap hardwareMap) {
        servo_sort = hardwareMap.get(Servo.class, "sort");
        servo_drop = hardwareMap.get(Servo.class, "drop");

        servo_sort.setPosition(sort_return);
        servo_drop.setPosition(drop_return);

        globals = new Globals();

        setTarget();
        populate_current();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }
    public void populate_current() {
        current_order[0] = -1;
        current_order[1] = -1;
        current_order[2] = -1;
    }
    public int returnTarget(int k) {
        return target_order[k];
    }
    public int returnCurrent(int k) {
        return current_order[k];
    }
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
        servo_drop.setPosition(drop_upwards);
    }
}
