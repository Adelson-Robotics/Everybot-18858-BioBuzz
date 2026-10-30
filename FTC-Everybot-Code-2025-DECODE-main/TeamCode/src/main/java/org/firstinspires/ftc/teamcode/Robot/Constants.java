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
    public static String LIMELIGHT_NAME      = "limelight";

    // INTAKE
    public static double INTAKE_IN_POWER  = 1.0;
    public static double INTAKE_OUT_POWER = -1.0;
    public static double INTAKE_OFF_POWER = 0.0;

    // CATAPULT
    public static double CATAPULT_UP_POWER   = -1.0;
    public static double CATAPULT_DOWN_POWER = 1.0;
    public static double CATAPULT_HOLD_POWER = 0.0;

    // ARM (placeholders, tune with ARM CALIBRATION)
    public static double ARM_UP_POS   = 0.575;
    public static double ARM_DOWN_POS = 0.84;

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

    // LIMELIGHT (pose only corrected while the robot is nearly still)
    public static double LIMELIGHT_MAX_LINEAR_SPEED  = 1.8;
    public static double LIMELIGHT_MAX_ANGULAR_SPEED = 1.8;

    // RED + BLUE TELEOP (no pose from auto, so they always start here)
    public static Pose TELEOP_DEFAULT_START_POSE = new Pose(9, 9, 0);

    // RED TELEOP (Y drives to whichever setpoint is closer)
    public static Pose RED_TARGET_SETPOINT_1 =
            new Pose(61.80, 10.17, Math.toRadians(90));
    // placeholder: old red goal setpoint, tune on the field
    public static Pose RED_TARGET_SETPOINT_2 =
            new Pose(61.80, 129.74, Math.toRadians(270));

    // BLUE TELEOP (Y drives to whichever setpoint is closer)
    // old blue goal setpoint was (27.02, 125.25, 142)
    public static Pose BLUE_TARGET_SETPOINT_1 =
            new Pose(84.7, 132.1, Math.toRadians(90));
    // placeholder: RED_TARGET_SETPOINT_1 mirrored, tune on the field
    public static Pose BLUE_TARGET_SETPOINT_2 =
            new Pose(85.4, 15.3, Math.toRadians(90));

    // ALL AUTOS
    public static double AUTO_CATAPULT_HOLD_POWER     = 0.15;
    public static double AUTO_LAUNCH_UP_DURATION      = 0.25;
    public static double AUTO_LAUNCH_DOWN_DURATION    = 0.4;
    public static double AUTO_READY_MAX_LINEAR_SPEED  = 3;
    public static double AUTO_READY_MAX_ANGULAR_SPEED = 3;
    public static double AUTO_SLOW_DRIVE_POWER        = 0.5;
    public static double AUTO_GARDEN_TIMEOUT          = 3.0;
    public static double AUTO_SHOT_WAIT               = 0.25;
    public static double AUTO_FLOWER_WAIT             = 0.5;
}
