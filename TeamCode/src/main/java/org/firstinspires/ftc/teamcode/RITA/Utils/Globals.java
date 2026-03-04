package org.firstinspires.ftc.teamcode.RITA.Utils;

public class Globals {
    public static boolean start_transfer;
    public static boolean pre_spin;
    public static boolean hanging = false;
    public static boolean first_ball = false, second_ball = false, third_ball = false;

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
