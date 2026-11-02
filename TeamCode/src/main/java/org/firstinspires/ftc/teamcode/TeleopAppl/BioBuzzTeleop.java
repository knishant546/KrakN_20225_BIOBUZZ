/* ============================================================================
 * Class: BioBuzzTeleop
 * What it does:
 *   Runs two-controller driving, intake/arm controls, and sequenced shooter/feeder operation.
 * Applications using it:
 *   BioBuzzTeleop (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TeleopAppl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.config.TeleopConfig.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeArmSingleServo;
import org.firstinspires.ftc.teamcode.subsystems.InputDCShooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterBBTeleop;

/** Driver-controlled chassis mode based on KKN_TEST's active mecanum test. */
@TeleOp(name = "BioBuzzTeleop", group = "BioBuzz")
public class BioBuzzTeleop extends OpMode {
    // Both motors initialize stopped during INIT.
    // Gamepad 2 START sequences the shooter and feeder after Driver Station START.
    private ShooterBBTeleop shooter;
    private boolean shooterAWasPressed, shooterBWasPressed;
    // Velocity control: start at 60% of 2600 ticks/s, with A/B limited to 0..85%.
    private int intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
    private boolean intakeXWasPressed, intakeYWasPressed;
    private boolean intakeReverse, intakeReverseXWasPressed;
    // A/B adjust speed by 5 percentage points per press, limited to 0..85%.
    private int selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
    private String lastShooterSpeedChange = "No speed change requested";
    private InputDCShooter inputDCShooter;
    private boolean feederStartWasPressed;
    private enum FeedSequence { IDLE, SPINNING_UP, FEEDING, SPINNING_DOWN }
    private FeedSequence feedSequence = FeedSequence.IDLE;
    private long sequenceDeadline;
    private boolean teleopStarted;

    private void stopShootingImmediately() {
        if (inputDCShooter != null) inputDCShooter.stop();
        if (shooter != null) shooter.stopImmediately();
        feedSequence = FeedSequence.IDLE;
        feederStatus = "Stopped";
    }
    private String feederStatus = "Ready / stopped";
    // Keep below MecanumDrive's 0.3 normalization floor so pure turning stays slower.
    private MecanumDrive driveTest;
    private Intake intake;
    private IntakeArmSingleServo intakeArm;
    private boolean armUpWasPressed, armDownWasPressed;
    private boolean intakeRunning, intakeManualStop;
    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    @Override public void init() {
        teleopStarted = false;
        feedSequence = FeedSequence.IDLE;
        shooter = ShooterBBTeleop.getInstance(); shooter.initialize(hardwareMap);
        selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
        lastShooterSpeedChange = "No speed change requested";
        driveTest = new MecanumDrive();
        // false selects the REV hub IMU named "imu"; true resets/configures Pinpoint.
        // Robot-relative drive() below does not use either sensor's heading.
        driveTest.init(hardwareMap, false);
        driveTest.drive(0, 0, 0);
        intake = new Intake();
        intake.init(hardwareMap); // Same intake hardware as KKN_TEST; running power is set below.
        intake.stop();
        intakeRunning = false;
        intakeManualStop = false;
        intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
        intakeReverse = false;
        intakeReverseXWasPressed = gamepad2.x;
        intakeXWasPressed = gamepad1.x;
        intakeYWasPressed = gamepad1.y;
        intakeArm = new IntakeArmSingleServo();
        intakeArm.init(hardwareMap); // Commands the configured UP_DEGREES during INIT.
        inputDCShooter = new InputDCShooter(); inputDCShooter.init(hardwareMap);
        inputDCShooter.setPower(FEEDER_POWER);
        feederStatus = "Ready / stopped";
        feederStartWasPressed = gamepad2.start;
        // MecanumDrive owns motor directions and encoder modes, matching KKN_TEST.
        frontLeft = hardwareMap.get(DcMotor.class, "FLDC");
        frontRight = hardwareMap.get(DcMotor.class, "FRDC");
        rearLeft = hardwareMap.get(DcMotor.class, "RLDC");
        rearRight = hardwareMap.get(DcMotor.class, "RRDC");
        telemetry.addLine("Ready: left stick drive/strafe, right stick X turn");
        telemetry.addLine("Shooter after Driver Station START: gamepad START sequence, BACK stop sequence, A +5%, B -5% (gamepad 2)");
        telemetry.addLine("Intake: auto-start on drive; D-pad RIGHT start/reset to 90% / LEFT stop (latched); Gamepad 1 X -10 / Y +10 percentage points; Gamepad 2 X reverses intake");
        telemetry.addLine("Arm: folded UP; after START: gamepad 1 UP/DOWN presets; gamepad 2 UP -5 / DOWN +5 degrees");
        telemetry.addData("Input shooter motor", "Ready / stopped; Gamepad 2 START: shooter then feeder; BACK: feeder then shooter");
    }

    @Override public void init_loop() {
        // Track buttons during INIT without starting either motor.
        updateInputDCShooter();
    }

    private void updateInputDCShooter() {
        long now = System.nanoTime();
        boolean startPressed = gamepad2.start && !feederStartWasPressed
                && !gamepad2.a && !gamepad2.b;
        // Stop requests win over START and cancel a pending feeder start.
        if (gamepad2.back) {
            if (inputDCShooter != null) inputDCShooter.stop();
            // Holding BACK must not continually restart the one-second timer.
            if (feedSequence == FeedSequence.SPINNING_UP || feedSequence == FeedSequence.FEEDING) {
                feedSequence = FeedSequence.SPINNING_DOWN;
                sequenceDeadline = now + (long) (SHOOTER_STOP_DELAY_SECONDS * 1e9);
                feederStatus = "Feeder stopped; shooter stops in " + SHOOTER_STOP_DELAY_SECONDS + " seconds";
            }
        } else if (startPressed) {
            if (!teleopStarted) {
                feederStatus = "Ready - wait for Driver Station START";
            } else if (selectedShooterSpeedPercent <= 0) {
                feederStatus = "Cannot feed: selected shooter speed is zero";
            } else if (feedSequence == FeedSequence.IDLE) {
                shooter.runSpeedFraction(selectedShooterSpeedPercent / 100.0);
                feedSequence = FeedSequence.SPINNING_UP;
                sequenceDeadline = now + (long) (SHOOTER_START_DELAY_SECONDS * 1e9);
                feederStatus = "Shooter starting; feeder waits " + SHOOTER_START_DELAY_SECONDS + " seconds";
            }
            // START during an active sequence does not restart its timer or cancel stopping.
        }
        // Nonblocking timers: driving and button handling continue during each delay.
        if (feedSequence == FeedSequence.SPINNING_UP && now >= sequenceDeadline) {
            if (shooter.getTargetVelocity() > 0 && !gamepad2.back) {
                inputDCShooter.start();
                feedSequence = FeedSequence.FEEDING;
                feederStatus = "Shooter + feeder running";
            } else stopShootingImmediately();
        } else if (feedSequence == FeedSequence.SPINNING_DOWN && now >= sequenceDeadline) {
            stopShootingImmediately();
        }
        feederStartWasPressed = gamepad2.start;
        telemetry.addData("Input shooter motor", feederStatus);
        telemetry.addData("Shooting sequence", feedSequence);
        if (feedSequence == FeedSequence.SPINNING_UP || feedSequence == FeedSequence.SPINNING_DOWN)
            telemetry.addData("Sequence delay remaining (s)", "%.2f",
                    Math.max(0, (sequenceDeadline - now) / 1e9));
    }

    @Override public void loop() {
        updateShooter();
        updateInputDCShooter();
        // FTC calls loop() only after START. Arm priority: D-pad UP, then D-pad DOWN.
        // No button means retain the last arm position; motor controls remain separate.
        if (gamepad1.dpad_up) intakeArm.liftUp();
        else if (gamepad1.dpad_down) intakeArm.liftDown();
        // Gamepad 1 presets win; Gamepad 2 nudges once per press, not every loop.
        else if (gamepad2.dpad_up && !armUpWasPressed && !gamepad2.dpad_down)
            intakeArm.adjustDegrees(-ARM_STEP_DEGREES);
        else if (gamepad2.dpad_down && !armDownWasPressed && !gamepad2.dpad_up)
            intakeArm.adjustDegrees(ARM_STEP_DEGREES);
        armUpWasPressed = gamepad2.dpad_up;
        armDownWasPressed = gamepad2.dpad_down;
        telemetry.addData("Intake arm target (deg)", intakeArm.getCommandedDegrees());
        // Robot-relative controls: left stick Y drives forward/backward (negated because

        //  Left stick up/down: drives forward/backward.
        //  Left stick/right: strafes the robot sideways left/right, keeping its facing direction.
        //  Right stick left/right: rotates the robot anticlockwise/clockwise.
        //  dpad right - Restore intake power to 90% and start Intake
        //  dpad left - Manually stop Intake
        //  Intake will automatically start once robo moves
        // Use Dpad left to stop Intake


        // Movement follows the robot's current front/right, not fixed field directions;
        // drive() does not use IMU or Pinpoint heading to compensate for robot rotation.
        // Ignore stick drift and square the remaining input for gentle center response.
        // Preserve direction; releasing the stick immediately commands zero turn.
        double stickTurn = gamepad1.right_stick_x;
        double magnitude = Math.max(0, (Math.abs(stickTurn) - TURN_DEADBAND)
                / (1 - TURN_DEADBAND));
        double turn = Math.copySign(MAX_TURN_INPUT * magnitude * magnitude, stickTurn);
        driveTest.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);
        // Start on any driving axis, ignoring small stick drift. Right-stick Y is unused.
        boolean driving = Math.abs(gamepad1.left_stick_y) > TURN_DEADBAND
                || Math.abs(gamepad1.left_stick_x) > TURN_DEADBAND
                || Math.abs(stickTurn) > TURN_DEADBAND;
        // Manual stop takes priority, even while driving or if both buttons are pressed.
        // Keep it latched so joystick input cannot undo the driver's stop command.
        if (gamepad1.dpad_left) {
            intakeManualStop = true;
            intakeRunning = false;
        } else if (gamepad1.dpad_right) {
            // RIGHT always restores the default 90% power, including after X reaches zero.
            intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
            intakeManualStop = false;
            intakeRunning = true; // Can also collect while the chassis is stationary.
        } else if (driving && !intakeManualStop) {
            intakeRunning = true;
        }
        // Releasing the sticks does not stop intake; use D-pad LEFT or OpMode STOP.
        // X/Y adjust once per press; RIGHT reset wins and simultaneous X+Y is ignored.
        if (gamepad1.x && !intakeXWasPressed && !gamepad1.y && !gamepad1.dpad_right)
            intakePowerPercent = Math.max(0, intakePowerPercent - INTAKE_STEP_PERCENT);
        else if (gamepad1.y && !intakeYWasPressed && !gamepad1.x && !gamepad1.dpad_right)
            intakePowerPercent = Math.min(MAX_INTAKE_POWER_PERCENT, intakePowerPercent + INTAKE_STEP_PERCENT);
        intakeXWasPressed = gamepad1.x;
        intakeYWasPressed = gamepad1.y;
        // Gamepad 2 X toggles direction once per press; it does not start a stopped intake.
        if (gamepad2.x && !intakeReverseXWasPressed) intakeReverse = !intakeReverse;
        intakeReverseXWasPressed = gamepad2.x;
        if (intakeRunning) intake.start(intakePowerPercent / 100.0, intakeReverse);
        else intake.stop();
        telemetry.addData("Intake", intakeManualStop ? "Manual STOP (RIGHT to restart)"
                : intakePowerPercent == 0 ? "Stopped (0% selected power)" : intakeRunning ? "Running" : "Waiting for drive or D-pad RIGHT");
        telemetry.addData("Intake selected power", "%d%% (X/Y step: %d points)", intakePowerPercent, INTAKE_STEP_PERCENT);
        telemetry.addData("Intake direction (gamepad 2 X toggles)", intakeReverse ? "REVERSE" : "NORMAL" );
        telemetry.addData("Intake decrease (gamepad 1 X)", gamepad1.x ? "PRESSED" : "Released");
        telemetry.addData("Intake increase (gamepad 1 Y)", gamepad1.y ? "PRESSED" : "Released");
        telemetry.addData("Turn input", "%.2f", turn);
        telemetry.addData("Drive", "Robot-relative");
        telemetry.addData("Front left power (FLDC)", "%.2f", frontLeft.getPower());
        telemetry.addData("Front right power (FRDC)", "%.2f", frontRight.getPower());
        telemetry.addData("Rear left power (RLDC)", "%.2f", rearLeft.getPower());
        telemetry.addData("Rear right power (RRDC)", "%.2f", rearRight.getPower());
    }

    @Override public void start() {
        teleopStarted = true;
        armUpWasPressed = gamepad2.dpad_up;
        armDownWasPressed = gamepad2.dpad_down;
        intakeReverseXWasPressed = gamepad2.x; // Ignore buttons held during INIT.
        intakeXWasPressed = gamepad1.x;
        intakeYWasPressed = gamepad1.y; // Ignore X held during INIT.
        feederStartWasPressed = gamepad2.start;
        // Require new presses after Driver Station START; ignore buttons held in INIT.
        shooterAWasPressed = gamepad2.a;
        shooterBWasPressed = gamepad2.b;
    }

    private void updateShooter() {
        // Ignore pairing chords and simultaneous A+B; holding a button changes once.
        boolean increasePressed = gamepad2.a && !shooterAWasPressed && !gamepad2.b
                && !gamepad2.start && !gamepad2.back;
        boolean decreasePressed = gamepad2.b && !shooterBWasPressed && !gamepad2.a
                && !gamepad2.start && !gamepad2.back;
        boolean speedChanged = increasePressed || decreasePressed;

        // Retain selected speed between sequences; INIT resets it to 60%.
        if (speedChanged) {
            int previousSpeed = selectedShooterSpeedPercent;
            int step = SHOOTER_STEP_PERCENT;
            selectedShooterSpeedPercent = Math.max(0, Math.min(MAX_SHOOTER_SPEED_PERCENT,
                    selectedShooterSpeedPercent + (increasePressed ? step : -step)));
            lastShooterSpeedChange = (increasePressed ? "A" : "B") + " pressed: "
                    + previousSpeed + " -> " + selectedShooterSpeedPercent
                    + "%" + (previousSpeed == selectedShooterSpeedPercent ? " (limit reached)" : "");
        }
        // A/B adjust running speed but never start a stopped shooter.
        // Zero speed also stops the feeder to avoid feeding into a stopped wheel.
        if (speedChanged && selectedShooterSpeedPercent <= 0) {
            stopShootingImmediately();
        } else if (speedChanged && shooter.getTargetVelocity() > 0
                && feedSequence != FeedSequence.SPINNING_DOWN) {
            shooter.runSpeedFraction(selectedShooterSpeedPercent / 100.0);
        }
        double target = shooter.getTargetVelocity();
        double measured = shooter.measuredVelocity();
        telemetry.addData("Shooter target (ticks/s)", "%.0f", target);
        telemetry.addData("Shooter measured (ticks/s)", "%.0f", measured);
        telemetry.addData("Shooter error (ticks/s)", "%.0f", target - measured);
        telemetry.addData("Shooter", target > 0 ? "Running (velocity control)" : "Stopped (coasts)");
        telemetry.addData("Shooter selected speed (% of reference)", "%d%%", selectedShooterSpeedPercent);
        telemetry.addData("Shooter SDK power (not measured RPM)", "%.0f%%", shooter.getShooterPower() * 100);
        telemetry.addData("Shooter gamepad 2 increase (A) / decrease (B)", "%s / %s",
                gamepad2.a ? "PRESSED" : "Released", gamepad2.b ? "PRESSED" : "Released");
        telemetry.addData("Shooter last speed change", lastShooterSpeedChange);
        shooterAWasPressed = gamepad2.a;
        shooterBWasPressed = gamepad2.b;
    }

    @Override public void stop() {
        stopShootingImmediately();
        teleopStarted = false;
        if (intake != null) intake.stop();
        if (driveTest != null) driveTest.drive(0, 0, 0);
        // KKN_TEST's inactive color sensor is not initialized or dereferenced here.
    }
}
