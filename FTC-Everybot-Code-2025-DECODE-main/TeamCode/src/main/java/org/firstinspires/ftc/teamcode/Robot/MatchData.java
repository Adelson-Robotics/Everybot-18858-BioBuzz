package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;

// Static fields carry over from Auto to TeleOp until the Robot Controller app restarts.
public class MatchData {
    public static Alliance lastAutoAlliance = null;
    public static Pose lastAutoPose = null;

    public static void saveFromAuto(Alliance alliance, Pose pose) {
        lastAutoAlliance = alliance;
        lastAutoPose = pose;
    }
}
