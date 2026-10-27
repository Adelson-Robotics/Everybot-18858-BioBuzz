package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Limelight.LimelightPoseUpdater;
import org.firstinspires.ftc.teamcode.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Catapult;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.MatchData;
import org.firstinspires.ftc.teamcode.Robot.RobotHardware;

// Controls: sticks drive, RT shoot, LT/LB intake in/out, Y auto-shot (sticks cancel), D-pad arm up/down
public abstract class BaseTeleop extends OpMode {

    private enum State { STARTUP, MANUAL_DRIVE, NAVIGATING_TO_TARGET, LAUNCHING }

    private RobotHardware robot;
    private Alliance alliance;
    private Pose targetSetpoint;

    private State state = State.STARTUP;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private boolean holdingForAutoShot = false;

    protected abstract Alliance chooseAlliance();

    @Override
    public void init() {
        alliance = chooseAlliance();
        targetSetpoint = alliance.targetSetpoint();

        robot = new RobotHardware(hardwareMap, false);
        robot.follower.setStartingPose(MatchData.lastAutoPose == null ? new Pose() : MatchData.lastAutoPose);
        robot.follower.update();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.startTeleopDrive();
        robot.limelight.start();
        robot.catapult.down();
        changeState(State.STARTUP);
    }

    @Override
    public void loop() {
        robot.follower.update();
        updatePoseFromLimelight();

        switch (state) {
            case STARTUP:              handleStartup(); break;
            case MANUAL_DRIVE:         handleManualDrive(); break;
            case NAVIGATING_TO_TARGET: handleNavigatingToTarget(); break;
            case LAUNCHING:            handleLaunching(); break;
        }

        robot.catapult.update();
        handleIntake();
        handleArm();
        updateTelemetry();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    private void handleStartup() {
        drive();
        if (stateTimer.seconds() >= Constants.STARTUP_DOWN_DURATION) {
            robot.catapult.hold();
            changeState(State.MANUAL_DRIVE);
        }
    }

    private void handleManualDrive() {
        if (gamepad1.right_trigger > Constants.TRIGGER_THRESHOLD) {
            startLaunch(false);
            return;
        }
        if (gamepad1.y) {
            startNavigationToTarget();
            return;
        }
        drive();
    }

    private void handleNavigatingToTarget() {
        if (isDriverMovingSticks()) {
            robot.follower.startTeleopDrive();
            changeState(State.MANUAL_DRIVE);
            return;
        }
        if (isReadyToLaunch()) {
            startLaunch(true);
        }
    }

    private void handleLaunching() {
        if (holdingForAutoShot && robot.catapult.getMode() != Catapult.Mode.UP) {
            robot.follower.startTeleopDrive();
            holdingForAutoShot = false;
        }
        if (!holdingForAutoShot) {
            drive();
        }
        if (!robot.catapult.isBusy()) {
            changeState(State.MANUAL_DRIVE);
        }
    }

    private void startLaunch(boolean fromTarget) {
        holdingForAutoShot = fromTarget;
        robot.catapult.launch();
        changeState(State.LAUNCHING);
    }

    private void startNavigationToTarget() {
        PathChain path = robot.follower.pathBuilder()
                .addPath(new Path(new BezierLine(robot.follower::getPose, targetSetpoint)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        robot.follower::getHeading, targetSetpoint.getHeading(), 0.1))
                .build();
        robot.follower.followPath(path);
        changeState(State.NAVIGATING_TO_TARGET);
    }

    private void drive() {
        double sign = alliance.driveSign();
        robot.follower.setTeleOpDrive(
                sign * gamepad1.left_stick_y,
                sign * gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
    }

    private void handleIntake() {
        boolean launching = state == State.LAUNCHING;
        if (gamepad1.left_trigger > Constants.TRIGGER_THRESHOLD) {
            robot.intake.in();
        } else if (gamepad1.left_bumper && !launching) {
            robot.intake.out();
        } else {
            robot.intake.off();
        }
    }

    private void handleArm() {
        if (gamepad1.dpad_up) {
            robot.arm.up();
        } else if (gamepad1.dpad_down) {
            robot.arm.down();
        }
    }

    // Skipped while a path is running, since a pose jump would make the follower jerk.
    private void updatePoseFromLimelight() {
        if (state == State.NAVIGATING_TO_TARGET || holdingForAutoShot) return;
        if (robot.getLinearSpeed() >= Constants.LIMELIGHT_MAX_LINEAR_SPEED
                || robot.getAngularSpeed() >= Constants.LIMELIGHT_MAX_ANGULAR_SPEED) return;

        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getBotposeTagCount() == 0) return;
        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;

        robot.follower.setPose(LimelightPoseUpdater.convertLimelightToPedro(
                botpose.getPosition().x,
                botpose.getPosition().y,
                robot.follower.getPose().getHeading()));
    }

    private void changeState(State newState) {
        state = newState;
        stateTimer.reset();
    }

    private boolean isDriverMovingSticks() {
        return Math.abs(gamepad1.left_stick_y) > Constants.DRIVER_DEADZONE
                || Math.abs(gamepad1.left_stick_x) > Constants.DRIVER_DEADZONE
                || Math.abs(gamepad1.right_stick_x) > Constants.DRIVER_DEADZONE;
    }

    private double distanceToTarget() {
        Pose pose = robot.follower.getPose();
        return Math.hypot(pose.getX() - targetSetpoint.getX(), pose.getY() - targetSetpoint.getY());
    }

    private boolean isReadyToLaunch() {
        double rawErr = robot.follower.getPose().getHeading() - targetSetpoint.getHeading();
        double headingErr = Math.atan2(Math.sin(rawErr), Math.cos(rawErr));
        return distanceToTarget() < Constants.SETPOINT_TOLERANCE
                && Math.abs(headingErr) < Constants.LAUNCH_HEADING_TOLERANCE
                && robot.getLinearSpeed() < Constants.LAUNCH_MAX_LINEAR_SPEED
                && robot.getAngularSpeed() < Constants.LAUNCH_MAX_ANGULAR_SPEED;
    }

    private void updateTelemetry() {
        Pose pose = robot.follower.getPose();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("State", state);
        telemetry.addData("Catapult", robot.catapult.getMode());
        telemetry.addData("Intake Power", "%.2f", robot.intake.getPower());
        telemetry.addData("Arm L / R", "%.3f / %.3f", robot.arm.getLeftPosition(), robot.arm.getRightPosition());
        telemetry.addData("Distance to Target", "%.2f", distanceToTarget());
        telemetry.addData("Pose", "X:%.2f Y:%.2f H:%.2f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}
