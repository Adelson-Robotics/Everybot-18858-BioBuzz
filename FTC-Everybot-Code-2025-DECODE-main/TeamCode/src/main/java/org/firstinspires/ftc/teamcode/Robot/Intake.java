package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, Constants.INTAKE_NAME);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void in()  { motor.setPower(Constants.INTAKE_IN_POWER); }
    public void out() { motor.setPower(Constants.INTAKE_OUT_POWER); }
    public void off() { motor.setPower(Constants.INTAKE_OFF_POWER); }

    public double getPower() { return motor.getPower(); }
}
