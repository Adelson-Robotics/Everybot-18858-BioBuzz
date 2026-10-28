package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Self-contained motor config checker. Doesn't use Robot/ or pedroPathing/ classes,
// so it works even if the rest of the robot code is broken.
// Picks one motor at a time so you can see on the robot which one actually spins.

@TeleOp(name = "MOTOR CONFIG TEST", group = "Util")
public class MotorConfigTest extends OpMode {

    // Names copied from Robot/Constants.java and pedroPathing/Constants.java
    private static final String[] MOTOR_NAMES = {
            "front_left",
            "front_right",
            "back_left",
            "back_right",
            "left_catapult",
            "right_catapult",
            "intake",
    };

    // Where each motor is supposed to be on the robot
    private static final String[] MOTOR_DESCRIPTIONS = {
            "Drive - FRONT LEFT wheel",
            "Drive - FRONT RIGHT wheel",
            "Drive - BACK LEFT wheel",
            "Drive - BACK RIGHT wheel",
            "Catapult - LEFT side motor",
            "Catapult - RIGHT side motor",
            "Intake roller motor",
    };

    private static final double POWER_STEP = 0.1;
    private static final double MAX_POWER = 1.0;

    private final DcMotor[] motors = new DcMotor[MOTOR_NAMES.length];
    private int selected = 0;
    private double testPower = 0.3; // low on purpose, the catapult can slam into its stops

    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
    private boolean wasRightBumperPressed = false;
    private boolean wasLeftBumperPressed = false;

    @Override
    public void init() {
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            // tryGet returns null instead of crashing if the name isn't in the config
            motors[i] = hardwareMap.tryGet(DcMotor.class, MOTOR_NAMES[i]);
            if (motors[i] != null) {
                motors[i].setDirection(DcMotorSimple.Direction.FORWARD); // raw, no code reversal
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setPower(0);
            }
        }

        telemetry.addLine("MOTOR CONFIG TEST ready");
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            telemetry.addData(MOTOR_NAMES[i], motors[i] == null ? "NOT FOUND IN CONFIG" : "found");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // DPAD UP/DOWN: pick a motor (stops everything when switching)
        if (gamepad1.dpad_down && !wasDpadDownPressed) selectMotor(selected + 1);
        if (gamepad1.dpad_up && !wasDpadUpPressed) selectMotor(selected - 1);
        wasDpadUpPressed = gamepad1.dpad_up;
        wasDpadDownPressed = gamepad1.dpad_down;

        // BUMPERS: change the A/B test power
        if (gamepad1.right_bumper && !wasRightBumperPressed) testPower = Math.min(MAX_POWER, testPower + POWER_STEP);
        if (gamepad1.left_bumper && !wasLeftBumperPressed) testPower = Math.max(POWER_STEP, testPower - POWER_STEP);
        wasRightBumperPressed = gamepad1.right_bumper;
        wasLeftBumperPressed = gamepad1.left_bumper;

        // A = forward at test power, B = reverse at test power, else left stick Y
        double power;
        if (gamepad1.a) power = testPower;
        else if (gamepad1.b) power = -testPower;
        else power = -gamepad1.left_stick_y * testPower;

        DcMotor motor = motors[selected];
        if (motor != null) motor.setPower(power);

        telemetry.addLine("=== MOTOR CONFIG TEST ===");
        telemetry.addLine("DPAD UP/DOWN: pick motor | A: fwd | B: rev");
        telemetry.addLine("LEFT STICK Y: variable | BUMPERS: test power +/-");
        telemetry.addLine();
        telemetry.addData("Selected", "%d/%d  \"%s\"", selected + 1, MOTOR_NAMES.length, MOTOR_NAMES[selected]);
        telemetry.addData("SHOULD MOVE", MOTOR_DESCRIPTIONS[selected]);

        if (motor == null) {
            telemetry.addLine("!! \"" + MOTOR_NAMES[selected] + "\" is NOT in the active config !!");
        } else {
            telemetry.addData("Port", motor.getPortNumber());
            telemetry.addData("Hub", motor.getController().getConnectionInfo());
            telemetry.addData("Power", "%.2f  (test power %.1f)", power, testPower);
            telemetry.addData("Encoder", motor.getCurrentPosition());
        }

        telemetry.addLine();
        telemetry.addLine("--- all motors ---");
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            String marker = (i == selected) ? ">> " : "   ";
            if (motors[i] == null) {
                telemetry.addLine(marker + MOTOR_NAMES[i] + ": NOT FOUND");
            } else {
                telemetry.addLine(marker + MOTOR_NAMES[i] + ": port " + motors[i].getPortNumber()
                        + ", enc " + motors[i].getCurrentPosition());
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        stopAll();
    }

    private void selectMotor(int index) {
        stopAll();
        selected = (index + MOTOR_NAMES.length) % MOTOR_NAMES.length;
    }

    private void stopAll() {
        for (DcMotor m : motors) {
            if (m != null) m.setPower(0);
        }
    }
}
