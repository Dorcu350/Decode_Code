package org.firstinspires.ftc.teamcode.StopC.Utils;

public class Globals {
    public static boolean start_feeding;
    public static boolean pre_spin;
    public static boolean has_shot;
    public static boolean has_shot_third;
    public static boolean auto_lock;
    public static double heading_error;

    //ENUMS

//
    public enum ALLIANCE {
        RED,
        BLUE
    }
    public static ALLIANCE alliance;

    public enum FAZE {
        AUTO,
        TELEOP
    }
    public static FAZE faze;
}
