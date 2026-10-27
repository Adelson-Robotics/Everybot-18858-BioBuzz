package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;

public enum Alliance {
    RED,
    BLUE;

    public Pose targetSetpoint() {
        return this == RED ? Constants.RED_TARGET_SETPOINT : Constants.BLUE_TARGET_SETPOINT;
    }

    // flips drive direction 
    public double driveSign() {
        return this == RED ? -1 : 1;
    }
}
