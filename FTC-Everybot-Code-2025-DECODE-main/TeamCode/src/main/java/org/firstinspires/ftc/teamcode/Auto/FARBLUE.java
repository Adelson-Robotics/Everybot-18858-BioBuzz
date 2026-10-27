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

@Autonomous(name = "BLUEFAR3", group = "Auto")
public class FARBLUE extends OpMode {

    private RobotHardware robot;
    private Follower follower;
    private Timer waitTimer;

    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    private Timer gateWaitTimer;

    private int pathState;
    private final Pose startPose = new Pose(80.29721362229103, 8, Math.toRadians(90)).mirror();
    private final Pose scorePose = new Pose(117.03426276788858, 124.3634424548243, Math.toRadians(40)).mirror();
    private final Pose endPose = new Pose(85.05727554179568,128.61300309597522,Math.toRadians(90)).mirror();

    private final Pose pickup1Pose = new Pose(80.52012383900929, 127.92569659442724, Math.toRadians(90)).mirror();
    private final Pose pickup1EndPose = new Pose(80.52012383900929, 127.92569659442724, Math.toRadians(90)).mirror();
    private final Pose pickup1AvoidPose = new Pose(92.76049766718508, 66.14618973561431, Math.toRadians(0)).mirror();
    private final Pose pickup2Pose = new Pose(98,62, Math.toRadians(0)).mirror();
    private final Pose pickup2EndPose = new Pose(126, 62, Math.toRadians(0)).mirror();
    private final Pose pickup2AvoidPose = new Pose(72, 84, Math.toRadians(0)).mirror();

    private final Pose pickup3Pose = new Pose(89, 40, Math.toRadians(0)).mirror();
    private final Pose pickup3EndPose = new Pose(126, 40, Math.toRadians(0)).mirror();

    private final Pose gateStartPose = new Pose(98, 67, Math.toRadians(0)).mirror();
    private final Pose gateEndPose = new Pose(85.05727554179568, 74, Math.toRadians(360)).mirror();
    private final Pose gateAndIntakePose = new Pose(130.98911353032653, 61.44323483670298, Math.toRadians(15.67)).mirror();
    private PathChain startToScore;
    private PathChain scoreToPickup2;
    private PathChain pickup2ToPickup2End;
    private PathChain pickup2EndToGateEnd;
    private PathChain gateEndToScore;
    private PathChain scoreToGateAndIntake;
    private PathChain gateAndIntakeToScore;

    private PathChain  startToPickup1Pose;
    private PathChain scoreToPickup1, pickup1ToPickup1End, pickup1EndToScore;
    private PathChain scoreToPickup3, pickup3ToPickup3End, pickup3ToScore, scoreToEndPose;

    public void buildPaths() {
        startToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        scoreToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
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

        pickup3ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3EndPose, pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
                .build();
        scoreToPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2AvoidPose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        startToPickup1Pose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                waitTimer.resetTimer();
                break;

            case 1:
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(startToPickup1Pose, true);
                    waitTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy() && waitTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(pickup1EndToScore, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    robot.catapult.launch();
                setPathState(4);
                }
                break;

                case 4:
                if(!robot.catapult.isBusy()){
                    follower.followPath(scoreToEndPose, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(follower.isBusy())  {
                    robot.intake.off();
                    setPathState(-1);
                }

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
        MatchData.saveFromAuto(Alliance.BLUE, follower.getPose());
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        robot.stop();
        MatchData.saveFromAuto(Alliance.BLUE, follower.getPose());

    }
}
