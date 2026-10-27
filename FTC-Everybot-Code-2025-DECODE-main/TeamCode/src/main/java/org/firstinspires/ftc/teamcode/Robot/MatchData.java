package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;

// for now just holds alliance color and pose, but possibly might add a doglog equivlent 
public class MatchData {
    public static Alliance lastAutoAlliance = null;
    public static Pose lastAutoPose = null;

    public static void saveFromAuto(Alliance alliance, Pose pose) {
        lastAutoAlliance = alliance;
        lastAutoPose = pose;
    }
}
