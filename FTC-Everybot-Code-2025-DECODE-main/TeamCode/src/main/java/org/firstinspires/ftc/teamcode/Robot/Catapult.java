package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Catapult {
    public enum Mode { UP, DOWN, HOLD }

    private final DcMotor left;
    private final DcMotor right;
    private final boolean useAutoTimings;
    private final ElapsedTime timer = new ElapsedTime();
    private Mode mode = Mode.HOLD;

    public Catapult(HardwareMap hardwareMap, boolean useAutoTimings) {
        this.useAutoTimings = useAutoTimings;
        left = hardwareMap.get(DcMotor.class, Constants.LEFT_CATAPULT_NAME);
        right = hardwareMap.get(DcMotor.class, Constants.RIGHT_CATAPULT_NAME);
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hold();
    }

    public void launch() {
        if (isBusy()) return;
        setMode(Mode.UP);
    }

    public void update() {
        double upDuration = useAutoTimings ? Constants.AUTO_LAUNCH_UP_DURATION : Constants.LAUNCH_UP_DURATION;
        double downDuration = useAutoTimings ? Constants.AUTO_LAUNCH_DOWN_DURATION : Constants.LAUNCH_DOWN_DURATION;

        if (mode == Mode.UP && timer.seconds() >= upDuration) {
            setMode(Mode.DOWN);
        } else if (mode == Mode.DOWN && timer.seconds() >= downDuration) {
            setMode(Mode.HOLD);
        }
    }

    public void down() { setMode(Mode.DOWN); }

    public void hold() { setMode(Mode.HOLD); }

    public void stop() {
        mode = Mode.HOLD;
        setPower(0);
    }

    public boolean isBusy() { return mode != Mode.HOLD; }

    public Mode getMode() { return mode; }

    private void setMode(Mode newMode) {
        mode = newMode;
        timer.reset();
        switch (newMode) {
            case UP:   setPower(Constants.CATAPULT_UP_POWER); break;
            case DOWN: setPower(Constants.CATAPULT_DOWN_POWER); break;
            case HOLD: setPower(useAutoTimings ? Constants.AUTO_CATAPULT_HOLD_POWER : Constants.CATAPULT_HOLD_POWER); break;
        }
    }

    private void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }
}
