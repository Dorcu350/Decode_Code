package org.firstinspires.ftc.teamcode.StopC.Utils;

public class Globals {
    public static boolean start_feeding;
    public static boolean pre_spin;
    public static boolean hanging;
    public static boolean has_shot_third;
    public static boolean auto_lock;
    public static double heading_error;

    //
    public static double curr_x, curr_y, curr_heading;

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
