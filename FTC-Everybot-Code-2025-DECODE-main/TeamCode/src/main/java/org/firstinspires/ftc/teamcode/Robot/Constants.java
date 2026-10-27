package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Constants {

    // MOTOR NAMES, I WONDER WHAT THE CHANCES ARE IF ANYONE EVER SEES THIS. 
    public static String LEFT_CATAPULT_NAME  = "left_catapult";
    public static String RIGHT_CATAPULT_NAME = "right_catapult";
    public static String INTAKE_NAME         = "intake";
    public static String LEFT_ARM_NAME       = "left_flower";
    public static String RIGHT_ARM_NAME      = "right_flower";

    // INTAKE
    public static double INTAKE_IN_POWER  = -0.8;
    public static double INTAKE_OUT_POWER = 1.0;
    public static double INTAKE_OFF_POWER = 0.0;

    // CATAPULT
    public static double CATAPULT_UP_POWER   = -1.0;
    public static double CATAPULT_DOWN_POWER = 1.0;
    public static double CATAPULT_HOLD_POWER = 0.0;

    // ARM (placeholders, tune with ARM CALIBRATION)
    public static double ARM_UP_POS   = 0.5;
    public static double ARM_DOWN_POS = 0.2;

    // ALL TELEOPS
    public static double LAUNCH_UP_DURATION       = 0.2;
    public static double LAUNCH_DOWN_DURATION     = 0.4;
    public static double STARTUP_DOWN_DURATION    = 0.25;
    public static double SETPOINT_TOLERANCE       = 2;
    public static double LAUNCH_HEADING_TOLERANCE = Math.toRadians(5);
    public static double LAUNCH_MAX_LINEAR_SPEED  = 4.0;
    public static double LAUNCH_MAX_ANGULAR_SPEED = 0.7;
    public static double DRIVER_DEADZONE          = 0.1;
    public static double TRIGGER_THRESHOLD        = 0.2;

    // RED TELEOP
    public static Pose RED_TARGET_SETPOINT =
            new Pose(122.30486929977042, 123.12394012200939, Math.toRadians(40));

    // BLUE TELEOP
    public static Pose BLUE_TARGET_SETPOINT =
            new Pose(27.021671826625383, 125.25077399380805, Math.toRadians(142));

    // ALL AUTOS
    public static double AUTO_CATAPULT_HOLD_POWER     = 0.15;
    public static double AUTO_LAUNCH_UP_DURATION      = 0.25;
    public static double AUTO_LAUNCH_DOWN_DURATION    = 0.4;
    public static double AUTO_READY_MAX_LINEAR_SPEED  = 3;
    public static double AUTO_READY_MAX_ANGULAR_SPEED = 3;
}
