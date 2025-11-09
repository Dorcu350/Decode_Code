package org.firstinspires.ftc.teamcode.MESSI.Utils;

public class Globals {
    public boolean sorter_active = false;
    public boolean heading_lock = false;
    public enum MOTIF {
        PPG,
        PGP,
        GPP
    }
    public MOTIF motif;
    public Globals() {
       motif = MOTIF.PGP;
    }
    //ENUMS

//
//    public enum MODE {
//        AUTO,
//        TELE_OP
//    }
}
