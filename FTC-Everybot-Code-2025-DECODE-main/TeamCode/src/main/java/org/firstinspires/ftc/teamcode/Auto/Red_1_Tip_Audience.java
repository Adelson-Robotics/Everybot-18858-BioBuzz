package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.MatchData;
import org.firstinspires.ftc.teamcode.Robot.RobotHardware;

@Autonomous (name = "Red_1Tip_Audience", group = "what does this even do?")
public class Red_1_Tip_Audience extends OpMode {

    private RobotHardware robot;
    private Follower follower;

    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    private int pathState;
    private final Pose startPose = new Pose(55, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(61.80, 10.17, Math.toRadians(90));
    // launches
    private final Pose RotationPose = new Pose(14.11, 31.75, Math.toRadians(270));

    private final Pose GardenLineupPose = new Pose(8.5, 30.96, Math.toRadians(270));
    // drop flower arm
    // run intake
    private final Pose GardenPickupPose = new Pose(8.5, 14.35, Math.toRadians(270));
    // drive to this pose slower
    private final Pose FlowerAvoidance = new Pose(18.95, 50.7, Math.toRadians(270));

    private final Pose endPose = new Pose(12.8,95.3, Math.toRadians(270));




    private PathChain startToScore, scoreToRotation, rotationToGardenLineup;
    private PathChain gardenLineupToGardenPickup, gardenPickupToFlowerAvoidance, flowerAvoidanceToEnd;

    private PathChain buildLine(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    public void buildPaths() {
        startToScore = buildLine(startPose, scorePose);
        scoreToRotation = buildLine(scorePose, RotationPose);
        rotationToGardenLineup = buildLine(RotationPose, GardenLineupPose);
        gardenLineupToGardenPickup = buildLine(GardenLineupPose, GardenPickupPose);
        gardenPickupToFlowerAvoidance = buildLine(GardenPickupPose, FlowerAvoidance);
        flowerAvoidanceToEnd = buildLine(FlowerAvoidance, endPose);
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(startToScore, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    // launches
                    robot.catapult.launch();
                    setPathState(2);
                }
                break;

            case 2:
                if (robot.isShotFiredAndSettled()) {
                    follower.followPath(scoreToRotation, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(rotationToGardenLineup, true);
                    robot.arm.down();
                    setPathState(4);
                }
                break;

            case 4:
                // timeout in case it can't reach garden lineup
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > Constants.AUTO_GARDEN_TIMEOUT) {
                    // drop flower arm, run intake, drive to garden pickup slower
                    robot.intake.in();
                    follower.followPath(gardenLineupToGardenPickup, Constants.AUTO_SLOW_DRIVE_POWER, true);
                    setPathState(5);
                }
                break;

            case 5:
                // timeout in case it can't reach garden pickup
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > Constants.AUTO_GARDEN_TIMEOUT) {
                    follower.followPath(gardenPickupToFlowerAvoidance, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(flowerAvoidanceToEnd, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        robot.update();
        autonomousPathUpdate();
        telemetryM.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("catapult mode", robot.catapult.getMode());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("path state", pathState);
        telemetryM.debug("catapult mode", robot.catapult.getMode());
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new RobotHardware(hardwareMap, true);
        follower = robot.follower;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        MatchData.saveFromAuto(Alliance.RED, follower.getPose());
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        robot.stop();
        MatchData.saveFromAuto(Alliance.RED, follower.getPose());
    }
}
