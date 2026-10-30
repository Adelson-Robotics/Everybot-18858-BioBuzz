package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Robot.MatchData;

@TeleOp(name = "AUTO TELEOP", group = "Teleop")
public class AutoTeleop extends BaseTeleop {
    @Override
    protected Alliance chooseAlliance() {
        return MatchData.lastAutoAlliance == null ? Alliance.RED : MatchData.lastAutoAlliance;
    }
}
