package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


// "arm" is the flower thing
public class Arm {
    private final Servo left;
    private final Servo right;

    public Arm(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, Constants.LEFT_ARM_NAME);
        right = hardwareMap.get(Servo.class, Constants.RIGHT_ARM_NAME);
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
    }

    public void up()   { setPosition(Constants.ARM_UP_POS); }
    public void down() { setPosition(Constants.ARM_DOWN_POS); }

    public void setPosition(double position) {
        left.setPosition(position);
        right.setPosition(position);
    }

    public double getLeftPosition()  { return left.getPosition(); }
    public double getRightPosition() { return right.getPosition(); }
}
