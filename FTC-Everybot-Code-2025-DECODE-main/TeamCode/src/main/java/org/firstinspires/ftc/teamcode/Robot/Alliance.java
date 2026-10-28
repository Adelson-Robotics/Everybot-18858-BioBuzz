package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.geometry.Pose;

public enum Alliance {
    RED,
    BLUE;

    public Pose[] targetSetpoints() {
        return this == RED
                ? new Pose[] { Constants.RED_TARGET_SETPOINT_1, Constants.RED_TARGET_SETPOINT_2 }
                : new Pose[] { Constants.BLUE_TARGET_SETPOINT_1, Constants.BLUE_TARGET_SETPOINT_2 };
    }

    // flips drive direction
    public double driveSign() {
        return this == RED ? -1 : 1;
    }

    // heading the BACK button resets to, matches the old RED/BLUE teleops
    public double resetHeading() {
        return this == RED ? 0 : Math.toRadians(180);
    }
}
