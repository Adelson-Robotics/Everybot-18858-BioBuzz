package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Arm;

@TeleOp(name = "ARM CALIBRATION", group = "Util")
public class ArmCalibration extends OpMode {

    private static final double STEP = 0.025;

    private Arm arm;
    private double position = 0.5;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
    }

    @Override
    public void start() {
        arm.setPosition(position);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !wasDpadUpPressed) position += STEP;
        if (gamepad1.dpad_down && !wasDpadDownPressed) position -= STEP;
        wasDpadUpPressed = gamepad1.dpad_up;
        wasDpadDownPressed = gamepad1.dpad_down;

        position = Math.max(0.0, Math.min(1.0, position));
        arm.setPosition(position);

        telemetry.addData("Arm Position", "%.4f", position);
        telemetry.addLine("DPAD UP: +0.025 | DPAD DOWN: -0.025");
        telemetry.update();
    }
}
