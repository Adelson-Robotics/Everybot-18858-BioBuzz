package org.firstinspires.ftc.teamcode.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public final Follower follower;
    public final Intake intake;
    public final Catapult catapult;
    public final Arm arm;

    public RobotHardware(HardwareMap hardwareMap, boolean isAuto) {
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        catapult = new Catapult(hardwareMap, isAuto);
        arm = new Arm(hardwareMap);
    }

    public void update() {
        follower.update();
        catapult.update();
    }

    public double getLinearSpeed() {
        Vector v = follower.getVelocity();
        return Math.hypot(v.getXComponent(), v.getYComponent());
    }

    public double getAngularSpeed() {
        return Math.abs(follower.getAngularVelocity());
    }

    public boolean isShotFiredAndSettled() {
        return catapult.isBusy()
                && getLinearSpeed() < Constants.AUTO_READY_MAX_LINEAR_SPEED
                && getAngularSpeed() < Constants.AUTO_READY_MAX_ANGULAR_SPEED;
    }

    public void stop() {
        intake.off();
        catapult.stop();
    }
}
