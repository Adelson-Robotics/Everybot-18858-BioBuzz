package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Alliance;

@TeleOp(name = "RED TELEOP", group = "Teleop")
public class RedTeleop extends BaseTeleop {
    @Override
    protected Alliance chooseAlliance() {
        return Alliance.RED;
    }
}
