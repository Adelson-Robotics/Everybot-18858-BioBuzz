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

@Autonomous (name = "Red_2Tip_Audience", group = "what does this even do?")
public class Red_2_Tip_Audience extends OpMode {

    private RobotHardware robot;
    private Follower follower;

    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    private int pathState;
    private final Pose startPose = new Pose(55, 9, Math.toRadians(90));
    private final Pose AudienceShootPose = new Pose(61.80, 10.17, Math.toRadians(90));
    // launches
    private final Pose RotationPose = new Pose(14.11, 31.75, Math.toRadians(270));

    private final Pose GardenLineupPose = new Pose(7.75, 30.96, Math.toRadians(270));
    // drop flower arm
    // run intake
    private final Pose GardenPickupPose = new Pose(7.75, 14.35, Math.toRadians(270));
    // drive to this pose slower
    private final Pose FlowerAvoidance = new Pose(18.95, 50.7, Math.toRadians(270));

    private final Pose IntakeAndDriveStart = new Pose(28, 70, Math.toRadians(90));

    private final Pose IntakeAndDriveEnd = new Pose(28, 110, Math.toRadians(90));
// Stop intaking
    // lift arm
    private final Pose PrepareToShoot = new Pose(61, 105, Math.toRadians(270));


    private final Pose ScoringShootPose = new Pose(61.80, 129.74, Math.toRadians(270));
//shoot
    private final Pose FlowerLineup = new Pose(46.43, 110.57, Math.toRadians(90));
// flower arm down
    // intake on

    private final Pose flowerIntake = new Pose(46.43, 130, Math.toRadians(90));

    private final Pose shimmyRight = new Pose(46.43, 123, Math.toRadians(90));

    private final Pose shimmyLeft = new Pose(46.43, 130, Math.toRadians(90));
// shimmy right after this

    private final Pose prepareShoot2 = new Pose(61.80, 117, Math.toRadians(270));
//go to scoringshootpose and shoot catapult
//raise flower arm
    private final Pose AboutToPark = new Pose(54.069, 111, Math.toRadians(270));

    private final Pose endPose = new Pose(11.658,100, Math.toRadians(270));
    //stop intake



    private PathChain startToAudienceShoot, audienceShootToRotation, rotationToGardenLineup;
    private PathChain gardenLineupToGardenPickup, gardenPickupToFlowerAvoidance;
    private PathChain flowerAvoidanceToIntakeAndDriveStart, intakeAndDriveStartToEnd;
    private PathChain intakeAndDriveEndToPrepareToShoot, prepareToShootToScoringShoot;
    private PathChain scoringShootToFlowerLineup, flowerLineupToFlowerIntake;
    private PathChain flowerIntakeToShimmyRight, shimmyRightToShimmyLeft, shimmyLeftToShimmyRight;
    private PathChain shimmyRightToPrepareShoot2, prepareShoot2ToScoringShoot;
    private PathChain scoringShootToAboutToPark, aboutToParkToEnd;

    private PathChain buildLine(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    public void buildPaths() {
        startToAudienceShoot = buildLine(startPose, AudienceShootPose);
        audienceShootToRotation = buildLine(AudienceShootPose, RotationPose);
        rotationToGardenLineup = buildLine(RotationPose, GardenLineupPose);
        gardenLineupToGardenPickup = buildLine(GardenLineupPose, GardenPickupPose);
        gardenPickupToFlowerAvoidance = buildLine(GardenPickupPose, FlowerAvoidance);
        flowerAvoidanceToIntakeAndDriveStart = buildLine(FlowerAvoidance, IntakeAndDriveStart);
        intakeAndDriveStartToEnd = buildLine(IntakeAndDriveStart, IntakeAndDriveEnd);
        intakeAndDriveEndToPrepareToShoot = buildLine(IntakeAndDriveEnd, PrepareToShoot);
        prepareToShootToScoringShoot = buildLine(PrepareToShoot, ScoringShootPose);
        scoringShootToFlowerLineup = buildLine(ScoringShootPose, FlowerLineup);
        flowerLineupToFlowerIntake = buildLine(FlowerLineup, flowerIntake);
        flowerIntakeToShimmyRight = buildLine(flowerIntake, shimmyRight);
        shimmyRightToShimmyLeft = buildLine(shimmyRight, shimmyLeft);
        shimmyLeftToShimmyRight = buildLine(shimmyLeft, shimmyRight);
        shimmyRightToPrepareShoot2 = buildLine(shimmyRight, prepareShoot2);
        prepareShoot2ToScoringShoot = buildLine(prepareShoot2, ScoringShootPose);
        scoringShootToAboutToPark = buildLine(ScoringShootPose, AboutToPark);
        aboutToParkToEnd = buildLine(AboutToPark, endPose);
    }

    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(startToAudienceShoot, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                // wait before the shot
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_SHOT_WAIT) {
                    // launches
                    robot.catapult.launch();
                    setPathState(3);
                }
                break;

            case 3:
                if (robot.isShotFiredAndSettled()) {
                    setPathState(4);
                }
                break;

            case 4:
                // wait after the shot
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_SHOT_WAIT) {
                    follower.followPath(audienceShootToRotation, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(rotationToGardenLineup, true);
                    robot.arm.down();
                    setPathState(6);
                }
                break;

            case 6:
                // timeout in case it can't reach garden lineup
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > Constants.AUTO_GARDEN_TIMEOUT) {
                    // drop flower arm, run intake, drive to garden pickup slower
                    robot.intake.in();
                    follower.followPath(gardenLineupToGardenPickup, Constants.AUTO_SLOW_DRIVE_POWER, true);
                    setPathState(7);
                }
                break;

            case 7:
                // timeout in case it can't reach garden pickup
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > Constants.AUTO_GARDEN_TIMEOUT) {
                    follower.followPath(gardenPickupToFlowerAvoidance, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(flowerAvoidanceToIntakeAndDriveStart, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(intakeAndDriveStartToEnd, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // stop intaking, lift arm
                    robot.intake.off();
                    robot.arm.up();
                    follower.followPath(intakeAndDriveEndToPrepareToShoot, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(prepareToShootToScoringShoot, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    setPathState(13);
                }
                break;

            case 13:
                // wait before the shot
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_SHOT_WAIT) {
                    // shoot
                    robot.catapult.launch();
                    setPathState(14);
                }
                break;

            case 14:
                if (robot.isShotFiredAndSettled()) {
                    setPathState(15);
                }
                break;

            case 15:
                // wait after the shot
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_SHOT_WAIT) {
                    follower.followPath(scoringShootToFlowerLineup, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    // flower arm down, intake on
                    robot.arm.down();
                    robot.intake.in();
                    follower.followPath(flowerLineupToFlowerIntake, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;

            case 18:
                // wait at flower intake
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_FLOWER_WAIT) {
                    follower.followPath(flowerIntakeToShimmyRight, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(shimmyRightToShimmyLeft, true);
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    setPathState(21);
                }
                break;

            case 21:
                // wait at shimmy left
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_FLOWER_WAIT) {
                    // shimmy right after this
                    follower.followPath(shimmyLeftToShimmyRight, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    // second shimmy left
                    follower.followPath(shimmyRightToShimmyLeft, true);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    setPathState(24);
                }
                break;

            case 24:
                // wait at shimmy left
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_FLOWER_WAIT) {
                    // second shimmy right
                    follower.followPath(shimmyLeftToShimmyRight, true);
                    setPathState(25);
                }
                break;

            case 25:
                if (!follower.isBusy()) {
                    // third shimmy left
                    follower.followPath(shimmyRightToShimmyLeft, true);
                    setPathState(26);
                }
                break;

            case 26:
                if (!follower.isBusy()) {
                    setPathState(27);
                }
                break;

            case 27:
                // wait at shimmy left
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_FLOWER_WAIT) {
                    // third shimmy right
                    follower.followPath(shimmyLeftToShimmyRight, true);
                    setPathState(28);
                }
                break;

            case 28:
                if (!follower.isBusy()) {
                    follower.followPath(shimmyRightToPrepareShoot2, true);
                    setPathState(29);
                }
                break;

            case 29:
                if (!follower.isBusy()) {
                    // go to scoring shoot pose
                    follower.followPath(prepareShoot2ToScoringShoot, true);
                    setPathState(30);
                }
                break;

            case 30:
                if (!follower.isBusy()) {
                    setPathState(31);
                }
                break;

            case 31:
                // wait before the shot
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_SHOT_WAIT) {
                    // shoot catapult
                    robot.catapult.launch();
                    setPathState(32);
                }
                break;

            case 32:
                if (robot.isShotFiredAndSettled()) {
                    setPathState(33);
                }
                break;

            case 33:
                // wait after the shot
                if (pathTimer.getElapsedTimeSeconds() > Constants.AUTO_SHOT_WAIT) {
                    // raise flower arm
                    robot.arm.up();
                    follower.followPath(scoringShootToAboutToPark, true);
                    setPathState(34);
                }
                break;

            case 34:
                if (!follower.isBusy()) {
                    follower.followPath(aboutToParkToEnd, true);
                    setPathState(35);
                }
                break;

            case 35:
                if (!follower.isBusy()) {
                    // stop intake
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
