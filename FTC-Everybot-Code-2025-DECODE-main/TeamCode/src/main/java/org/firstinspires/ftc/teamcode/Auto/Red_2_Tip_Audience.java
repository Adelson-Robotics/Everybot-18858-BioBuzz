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
    private final Pose startPose = new Pose(55, 9, Math.toRadians(270));
    private final Pose AudienceShootPose = new Pose(61.80, 10.17, Math.toRadians(270));
    // launches
    private final Pose RotationPose = new Pose(14.11, 31.75, Math.toRadians(90));

    private final Pose GardenLineupPose = new Pose(10.38, 30.96, Math.toRadians(90));
    // drop flower arm
    // run intake
    private final Pose GardenPickupPose = new Pose(10.19, 14.35, Math.toRadians(90));
    // drive to this pose slower
    private final Pose FlowerAvoidance = new Pose(24, 50.7, Math.toRadians(90));

    private final Pose IntakeAndDriveStart = new Pose(41.4, 70, Math.toRadians(270));

    private final Pose IntakeAndDriveEnd = new Pose(41.4, 100, Math.toRadians(270));
// Stop intaking
    // lift arm
    private final Pose PrepareToShoot = new Pose(67.92, 107.56, Math.toRadians(270));


    private final Pose ScoringShootPose = new Pose(61.80, 129.74, Math.toRadians(270));
//shoot
    private final Pose FlowerLineup = new Pose(57.24, 109.29, Math.toRadians(90));
// flower arm down
    // intake on

    private final Pose flowerIntake = new Pose(58.92, 109.29, Math.toRadians(90));

    private final Pose shimmyRight = new Pose(61.35, 123, Math.toRadians(96));

    private final Pose shimmyLeft = new Pose(56.5, 123.5, Math.toRadians(74));
// shimmy right after this

    private final Pose prepareShoot2 = new Pose(69.21, 96.98, Math.toRadians(270));
//go to scoringshootpose and shoot catapult
    private final Pose AboutToPark = new Pose(69.21, 93.98, Math.toRadians(90));

    private final Pose endPose = new Pose(12.8,93.3, Math.toRadians(90));
    //raise flower arm
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
                    // launches
                    robot.catapult.launch();
                    setPathState(2);
                }
                break;

            case 2:
                if (robot.isShotFiredAndSettled()) {
                    follower.followPath(audienceShootToRotation, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(rotationToGardenLineup, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    // drop flower arm, run intake, drive to garden pickup slower
                    robot.arm.down();
                    robot.intake.in();
                    follower.followPath(gardenLineupToGardenPickup, Constants.AUTO_SLOW_DRIVE_POWER, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(gardenPickupToFlowerAvoidance, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(flowerAvoidanceToIntakeAndDriveStart, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(intakeAndDriveStartToEnd, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    // stop intaking, lift arm
                    robot.intake.off();
                    robot.arm.up();
                    follower.followPath(intakeAndDriveEndToPrepareToShoot, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(prepareToShootToScoringShoot, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // shoot
                    robot.catapult.launch();
                    setPathState(11);
                }
                break;

            case 11:
                if (robot.isShotFiredAndSettled()) {
                    follower.followPath(scoringShootToFlowerLineup, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    // flower arm down, intake on
                    robot.arm.down();
                    robot.intake.in();
                    follower.followPath(flowerLineupToFlowerIntake, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(flowerIntakeToShimmyRight, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(shimmyRightToShimmyLeft, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    // shimmy right after this
                    follower.followPath(shimmyLeftToShimmyRight, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(shimmyRightToPrepareShoot2, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    // go to scoring shoot pose
                    follower.followPath(prepareShoot2ToScoringShoot, true);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    // shoot catapult
                    robot.catapult.launch();
                    setPathState(19);
                }
                break;

            case 19:
                if (robot.isShotFiredAndSettled()) {
                    follower.followPath(scoringShootToAboutToPark, true);
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(aboutToParkToEnd, true);
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy()) {
                    // raise flower arm, stop intake
                    robot.arm.up();
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
