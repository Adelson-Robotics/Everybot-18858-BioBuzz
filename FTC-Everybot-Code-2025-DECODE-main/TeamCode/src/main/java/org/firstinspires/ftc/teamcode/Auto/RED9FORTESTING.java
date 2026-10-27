package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Robot.MatchData;
import org.firstinspires.ftc.teamcode.Robot.RobotHardware;

@Autonomous (name = "RED9PERCHANCE", group = "what does this even do?")
public class RED9FORTESTING extends OpMode {

    private RobotHardware robot;
    private Follower follower;
    private Timer waitTimer;

    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    private Timer gateWaitTimer;

    private int pathState;
    private final Pose startPose = new Pose(116.69051321928461, 132.29237947122863, Math.toRadians(37));
    private final Pose scorePose = new Pose(116.03426276788858, 125.3634424548243, Math.toRadians(37));
    private final Pose endPose = new Pose(110,132.29237, Math.toRadians(37));

    private final Pose scoreEndPose = new Pose(116.03426276788858, 125.3634424548243, Math.toRadians(37));
    private final Pose pickup1Pose = new Pose(89, 88.5, Math.toRadians(0));
    private final Pose pickup1EndPose = new Pose(123.193, 88.5, Math.toRadians(0));
    private final Pose pickup1AvoidPose = new Pose(92.76049766718508, 66.14618973561431, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(98,62, Math.toRadians(0));
    private final Pose pickup2EndPose = new Pose(126, 62, Math.toRadians(0));
    private final Pose pickup2AvoidPose = new Pose(72, 84, Math.toRadians(0));

    private final Pose Pickup3BackOff = new Pose(80, 43, Math.toRadians(60));

    private final Pose pickup3Pose = new Pose(91, 43, Math.toRadians(0));
    private final Pose pickup3EndPose = new Pose(126, 43, Math.toRadians(0));

    private final Pose gateStartPose = new Pose(98, 63, Math.toRadians(0));
    private final Pose gateEndPose = new Pose(121.7, 69.5, Math.toRadians(0));
    private final Pose gateAndIntakePose = new Pose(126, 65.44323483670298, Math.toRadians(26));

    private PathChain startToScore;
    private PathChain scoreToPickup2;
    private PathChain pickup2ToPickup2End;
    private PathChain pickup2EndToGateEnd;
    private PathChain gateEndToScore;
    private PathChain scoreToGateAndIntake;
    private PathChain gateAndIntakeToScore;
    private PathChain scoreToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End;
    private PathChain pickup3ToPickup3Start, pickup3ToScoreEnd, scoreEndToEndPose;

    public void buildPaths() {
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pickup2ToPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2EndPose.getHeading())
                .build();

        pickup2EndToGateEnd = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2EndPose, gateStartPose, gateEndPose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), gateEndPose.getHeading())
                .build();

        gateEndToScore = follower.pathBuilder()
                .addPath(new BezierCurve(gateEndPose, pickup1AvoidPose, pickup2AvoidPose, scorePose))
                .setLinearHeadingInterpolation(gateEndPose.getHeading(), scorePose.getHeading())
                .build();

        scoreToGateAndIntake = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2AvoidPose, pickup1AvoidPose, gateAndIntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateAndIntakePose.getHeading())
                .build();

        gateAndIntakeToScore = follower.pathBuilder()
                .addPath(new BezierCurve(gateAndIntakePose, pickup1AvoidPose, pickup2AvoidPose, scorePose))
                .setLinearHeadingInterpolation(gateAndIntakePose.getHeading(), scorePose.getHeading())
                .build();

        scoreToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        pickup1ToPickup1End = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1EndPose.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        pickup1EndToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        scoreToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        pickup3ToPickup3End = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup3EndPose.getHeading())
                .setVelocityConstraint(0.5)
                .build();

        pickup3ToPickup3Start = follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, Pickup3BackOff))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), Pickup3BackOff.getHeading())
                .build();

        pickup3ToScoreEnd = follower.pathBuilder()
                .addPath(new BezierLine(Pickup3BackOff, scoreEndPose))
                .setLinearHeadingInterpolation(Pickup3BackOff.getHeading(), scoreEndPose.getHeading(), 0.01)
                .build();

        scoreToPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2AvoidPose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scoreEndToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(scoreEndPose, endPose))
                .setLinearHeadingInterpolation(scoreEndPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(startToScore, true);
                robot.intake.in();
                setPathState(1);
                waitTimer.resetTimer();
                break;

            case 1:
                if (!follower.isBusy()) {
                    robot.catapult.launch();
                    waitTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                if (robot.isShotFiredAndSettled() && waitTimer.getElapsedTimeSeconds() > 0.2) {
                    robot.intake.in();
                    follower.followPath(scoreToPickup2, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2ToPickup2End, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickup2EndToGateEnd, 0.85, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    waitTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(gateEndToScore, true);
                    setPathState(7);
                    waitTimer.resetTimer();
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    robot.catapult.launch();
                    setPathState(8);
                    waitTimer.resetTimer();
                }
                break;

            case 8:
                if (robot.isShotFiredAndSettled() && waitTimer.getElapsedTimeSeconds() > 0.15) {
                    follower.followPath(scoreToGateAndIntake, true);
                    robot.intake.in();
                    waitTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 3.25) {
                    follower.followPath(gateAndIntakeToScore, true);
                    setPathState(10);
                    waitTimer.resetTimer();
                }
                break;

                case 10:
                if (!follower.isBusy()) {
                    robot.catapult.launch();
                    setPathState(11);
                    waitTimer.resetTimer();
                }
                break;

            case 11:
                if (robot.isShotFiredAndSettled() && waitTimer.getElapsedTimeSeconds() > 0.15) {
                    follower.followPath(scoreToGateAndIntake, true);
                    robot.intake.in();
                    waitTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 3.25) {
                    follower.followPath(gateAndIntakeToScore, true);
                    setPathState(13);
                    waitTimer.resetTimer();
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    robot.catapult.launch();
                    setPathState(14);
                }
                break;

            case 14:
                if (robot.isShotFiredAndSettled() && waitTimer.getElapsedTimeSeconds() > 0.15) {
                    follower.followPath(scoreToPickup1, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    robot.intake.in();
                    follower.followPath(pickup1ToPickup1End, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(17);
                    waitTimer.resetTimer();
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    robot.catapult.launch();
                    setPathState(18);
                }
                break;

            case 18:
                if (robot.isShotFiredAndSettled()) {
                    follower.followPath(scoreEndToEndPose, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.3) {
                    robot.intake.off();
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
        waitTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        gateWaitTimer = new Timer();
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
        robot.intake.in();
        setPathState(0);
    }

    @Override
    public void stop() {
        robot.stop();
        MatchData.saveFromAuto(Alliance.RED, follower.getPose());
    }
}