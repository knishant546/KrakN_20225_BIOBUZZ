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
import org.firstinspires.ftc.teamcode.subsystems.PollenShooter;


/** Driver-controlled chassis mode based on KKN_TEST's active mecanum test. */
@TeleOp(name = "BioBuzzTeleop", group = "BioBuzz")
public class BioBuzzTeleop extends OpMode {
    // Pollen and feeder initialize stopped during INIT; Nectar is disabled.
    // Gamepad 2 LT sequences the shooter and feeder after Driver Station START.
    private PollenShooter shooter;

    private boolean shooterRightWasPressed, shooterLeftWasPressed;
    // Velocity control uses TeleopConfig defaults; RIGHT/LEFT adjust the selected target.
    private int intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
    private boolean intakeLeftWasPressed, intakeRightWasPressed, intakeStartWasPressed;
    private boolean helperAWasPressed, helperBWasPressed;
    private int helperPowerPercent = INITIAL_INTAKE_HELPER_POWER_PERCENT;
    private boolean helperReverse, helperReverseXWasPressed;
    // D-pad RIGHT/LEFT adjust speed by 1 percentage point per press, limited to 0..85%.
    private int selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
    private String lastShooterSpeedChange = "No speed change requested";
    private InputDCShooter inputDCShooter;
    // Trigger depth selects a button press, not shooter velocity.
    private static final double SHOOTER_TRIGGER_THRESHOLD = 0.5;
    private boolean feederStartWasPressed;
    enum FeedSequence { IDLE, SPINNING_UP, RUNNING, RECOVERING }
    private FeedSequence feedSequence = FeedSequence.IDLE;
    private final ShooterFeedGate feedGate = new ShooterFeedGate();
    private boolean teleopStarted;

    private void stopShootingImmediately() {
        if (inputDCShooter != null) inputDCShooter.stop();
        if (shooter != null) shooter.stopImmediately();
        // Shooting controls never change the independent intake request.
        feedGate.stop();
        feedSequence = FeedSequence.IDLE;
        feederStatus = "Stopped";
    }
    private String feederStatus = "Ready / stopped";
    // Keep below MecanumDrive's 0.3 normalization floor so pure turning stays slower.
    private MecanumDrive driveTest;
    private Intake intake;
    private IntakeArmSingleServo intakeArm;

    private boolean intakeRunning, intakeManualStop;
    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    /** Variant hook: false skips helper hardware lookup and prevents helper movement. */
    protected boolean isIntakeHelperEnabled() { return false; }

    @Override public void init() {
        teleopStarted = false;
        feedSequence = FeedSequence.IDLE;
        shooter = PollenShooter.getInstance(); shooter.initialize(hardwareMap);
        feedGate.stop(); // Nectar is disabled: no hardware lookup or commands.
        selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
        lastShooterSpeedChange = "No speed change requested";
        driveTest = new MecanumDrive();
        // false selects the REV hub IMU named "imu"; true resets/configures Pinpoint.
        // Robot-relative drive() below does not use either sensor's heading.
        driveTest.init(hardwareMap, false);
        driveTest.drive(0, 0, 0);
        intake = new Intake();
        intake.init(hardwareMap, isIntakeHelperEnabled()); // Helper disabled: no hardware lookup or movement.
        intake.stop();
        helperPowerPercent = INITIAL_INTAKE_HELPER_POWER_PERCENT;
        intake.setHelperReverse(helperReverse);
        intake.setHelperPower(helperPowerPercent / 100.0);
        intakeRunning = false;
        intakeManualStop = false;
        intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
        helperReverse = false;
        helperReverseXWasPressed = gamepad2.x;
        intakeLeftWasPressed = gamepad1.dpad_left;
        intakeRightWasPressed = gamepad1.dpad_right;
        intakeStartWasPressed = gamepad1.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        intakeArm = new IntakeArmSingleServo();
        intakeArm.init(hardwareMap); // Commands the configured UP_DEGREES during INIT.
        inputDCShooter = new InputDCShooter();
        inputDCShooter.init(hardwareMap);
        // Match the corrected feeder direction tested in TestShootingCalibration.
        inputDCShooter.setPower(-FEEDER_POWER);
        feederStatus = "Ready / stopped";
        feederStartWasPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        // MecanumDrive owns motor directions and encoder modes, matching KKN_TEST.
        frontLeft = hardwareMap.get(DcMotor.class, "FLDC");
        frontRight = hardwareMap.get(DcMotor.class, "FRDC");
        rearLeft = hardwareMap.get(DcMotor.class, "RLDC");
        rearRight = hardwareMap.get(DcMotor.class, "RRDC");
        telemetry.addLine("Ready: left stick drive/strafe, right stick X turn");
        telemetry.addLine("Gamepad 2: LT starts Pollen sequence; RT stops Pollen/feeder; RIGHT/LEFT adjusts Pollen");
        telemetry.addLine("Intake G1: LT start / RT stop; D-pad RIGHT +1 / LEFT -1 point. Driving does not start intake.");
        telemetry.addLine("Arm: folded UP; after START: gamepad 1 UP/DOWN presets");
        telemetry.addData("Input shooter motor", "Enabled; waits for Pollen speed or startup timeout. Nectar disabled.");
    }

    @Override public void init_loop() {
        // Track buttons during INIT without starting either motor.
        updateInputDCShooter();
    }

    /** Speed gate does not block driving; startup timeout applies only to initial spin-up. */
    private void updateInputDCShooter() {
        double now = System.nanoTime() / 1e9;
        boolean startPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD && !feederStartWasPressed;
        if (gamepad2.right_trigger > SHOOTER_TRIGGER_THRESHOLD) {
            stopShootingImmediately();
        } else if (startPressed && teleopStarted && feedSequence == FeedSequence.IDLE) {
            if (selectedShooterSpeedPercent > 0) {
                inputDCShooter.stop();
                shooter.runSpeedFraction(selectedShooterSpeedPercent / 100.0);
                feedGate.start(now);
            } else feederStatus = "Cannot start: Pollen selected speed is zero";
        }
        feedGate.update(now, shooter.getTargetVelocity(), shooter.measuredVelocity());
        feedSequence = feedGate.state;
        // Feeder output follows this gate; intake is controlled independently by Gamepad 1.
        feederStartWasPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
    }

    /** Pure timing/velocity state machine, also exercised by the local regression check. */
    static final class ShooterFeedGate {
        FeedSequence state = FeedSequence.IDLE;
        double startedAt, lowSince = Double.NaN, readySince = Double.NaN;
        void start(double now) { stop(); startedAt = now; state = FeedSequence.SPINNING_UP; }
        void stop() { state = FeedSequence.IDLE; lowSince = readySince = Double.NaN; }
        void update(double now, double target, double measured) {
            if (state == FeedSequence.IDLE) return;
            if (target <= 0 || !Double.isFinite(target)) { stop(); return; }
            boolean finite = Double.isFinite(measured);
            double tolerance = Math.min(SHOOTER_READY_MAX_ERROR_TICKS, target * SHOOTER_READY_ERROR_FRACTION);
            boolean ready = finite && Math.abs(target - measured) <= tolerance;
            if (state == FeedSequence.SPINNING_UP) {
                if (ready || now - startedAt >= SHOOTER_START_TIMEOUT_SECONDS)
                    state = FeedSequence.RUNNING;
            } else if (state == FeedSequence.RUNNING) {
                boolean slow = !finite || measured < target * (1 - SHOOTER_DROP_FRACTION);
                if (!slow) lowSince = Double.NaN;
                else {
                    if (Double.isNaN(lowSince)) lowSince = now;
                    if (now - lowSince >= SHOOTER_DROP_HOLD_SECONDS) {
                        state = FeedSequence.RECOVERING;
                        readySince = Double.NaN;
                    }
                }
            } else if (state == FeedSequence.RECOVERING) {
                if (!ready) readySince = Double.NaN;
                else {
                    if (Double.isNaN(readySince)) readySince = now;
                    if (now - readySince >= SHOOTER_RECOVERY_HOLD_SECONDS) {
                        state = FeedSequence.RUNNING;
                        lowSince = Double.NaN;
                    }
                }
            }
        }
    }
    @Override public void loop() {
        updateShooter();

        updateInputDCShooter();
        // Ignore diagonal D-pad inputs: intake LEFT/RIGHT must never move the arm.
        // FTC calls loop() after START; otherwise retain the last commanded arm position.
        if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
            if (gamepad1.dpad_up && !gamepad1.dpad_down) intakeArm.liftUp();
            else if (gamepad1.dpad_down && !gamepad1.dpad_up) intakeArm.liftDown();
        }
        telemetry.addData("Intake arm target (deg)", intakeArm.getCommandedDegrees());
        // Robot-relative controls: left stick Y drives forward/backward (negated because

        //  Left stick up/down: drives forward/backward.
        //  Left stick/right: strafes the robot sideways left/right, keeping its facing direction.
        //  Right stick left/right: rotates the robot anticlockwise/clockwise.
        // Intake controls are independent of driving: Gamepad 1 LT/RT start/stop.


        // Movement follows the robot's current front/right, not fixed field directions;
        // drive() does not use IMU or Pinpoint heading to compensate for robot rotation.
        // Ignore stick drift and square the remaining input for gentle center response.
        // Preserve direction; releasing the stick immediately commands zero turn.
        double stickTurn = gamepad1.right_stick_x;
        double magnitude = Math.max(0, (Math.abs(stickTurn) - TURN_DEADBAND)
                / (1 - TURN_DEADBAND));
        // Compensate for the drive mixer scaling so TURN_MAX_POWER is wheel power.
        double maxTurnInput = DRIVE_MAX_POWER > 0.0
                ? Math.min(TURN_MAX_POWER, DRIVE_MAX_POWER) / DRIVE_MAX_POWER : 0.0;
        double turn = Math.copySign(maxTurnInput * magnitude * magnitude, stickTurn);
        // Shape translation independently: small stick movements give finer control.
        // Full stick retains the configured maximum; release commands zero immediately.
        driveTest.drive(shapeDriveInput(-gamepad1.left_stick_y),
                shapeDriveInput(gamepad1.left_stick_x), turn);
        // Intake starts only from Gamepad 1 LT, never from driving or shooting.
        // Gamepad 1 RT wins over LT; shooter speed never pauses intake.
        boolean intakeStartPressed = gamepad1.left_trigger > SHOOTER_TRIGGER_THRESHOLD && !intakeStartWasPressed;
        if (gamepad1.right_trigger > SHOOTER_TRIGGER_THRESHOLD) {
            intakeManualStop = true;
            intakeRunning = false;
        } else if (intakeStartPressed) {
            intakeManualStop = false;
            intakeRunning = true;
        }
        // Adjust once per press; changing speed does not itself start a stopped intake.
        if (gamepad1.dpad_left && !intakeLeftWasPressed && !gamepad1.dpad_right)
            intakePowerPercent = Math.max(0, intakePowerPercent - INTAKE_STEP_PERCENT);
        else if (gamepad1.dpad_right && !intakeRightWasPressed && !gamepad1.dpad_left)
            intakePowerPercent = Math.min(MAX_INTAKE_POWER_PERCENT, intakePowerPercent + INTAKE_STEP_PERCENT);
        intakeLeftWasPressed = gamepad1.dpad_left;
        intakeRightWasPressed = gamepad1.dpad_right;
        intakeStartWasPressed = gamepad1.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        // Gamepad 2 X toggles only the helper once per press; it never starts a stopped intake.
        if (isIntakeHelperEnabled() && gamepad2.x && !helperReverseXWasPressed) helperReverse = !helperReverse;
        helperReverseXWasPressed = gamepad2.x;
        // Independent helper selection: one adjustment per press, ignoring pairing chords.
        if (isIntakeHelperEnabled() && gamepad2.a && !helperAWasPressed && !gamepad2.b && !gamepad2.start && !gamepad2.back)
            helperPowerPercent = Math.min(100, helperPowerPercent + INTAKE_HELPER_STEP_PERCENT);
        else if (isIntakeHelperEnabled() && gamepad2.b && !helperBWasPressed && !gamepad2.a && !gamepad2.start && !gamepad2.back)
            helperPowerPercent = Math.max(0, helperPowerPercent - INTAKE_HELPER_STEP_PERCENT);
        helperAWasPressed = gamepad2.a;
        helperBWasPressed = gamepad2.b;
        intake.setHelperReverse(helperReverse);
        intake.setHelperPower(helperPowerPercent / 100.0);
        telemetry.addData("Intake helper selected power (G2 A+/B-)", "%d%%", helperPowerPercent);
        telemetry.addData("Intake helper", !isIntakeHelperEnabled() ? "DISABLED (no hardware initialized)" : intakeRunning && intakePowerPercent > 0 && helperPowerPercent > 0
                ? (helperReverse ? "Running reversed" : "Running") : "Stopped");
        // Keep collected balls held: shooter spin-up/recovery only pauses the feeder.
        // Gamepad 1 RT stops intake only; feeder remains governed by the shooting sequence.

        boolean intakeOutput = intakeRunning && !intakeManualStop && intakePowerPercent > 0;
        if (intakeOutput) intake.start(intakePowerPercent / 100.0, false, false);
        else intake.stop();
        if (feedSequence == FeedSequence.RUNNING) inputDCShooter.start();
        else inputDCShooter.stop();
        feederStatus = feedSequence == FeedSequence.RUNNING ? "Running"
                : feedSequence == FeedSequence.SPINNING_UP ? "Waiting for Pollen speed / startup timeout"
                : feedSequence == FeedSequence.RECOVERING ? "Paused: waiting for stable Pollen recovery" : "Stopped";
        telemetry.addData("Input shooter motor", feederStatus);
        telemetry.addData("Shooting sequence", feedSequence);
        telemetry.addData("Intake actual output", intakeOutput ? "Running (independent of shooter)" : "Stopped");
        telemetry.addData("Pollen ready tolerance (ticks/s)", "%.0f",
                Math.min(SHOOTER_READY_MAX_ERROR_TICKS, shooter.getTargetVelocity() * SHOOTER_READY_ERROR_FRACTION));
        telemetry.addData("Nectar", "DISABLED (no hardware initialized)");
        telemetry.addData("Intake", intakeManualStop ? "Manual STOP (LT to restart)"
                : intakePowerPercent == 0 ? "Stopped (0% selected power)" : intakeRunning ? "Running" : "Waiting for Gamepad 1 LT");
        telemetry.addData("Intake selected power", "%d%% (D-pad step: %d points)", intakePowerPercent, INTAKE_STEP_PERCENT);
        telemetry.addData("Helper direction (gamepad 2 X toggles)", !isIntakeHelperEnabled() ? "DISABLED" : helperReverse ? "REVERSE" : "NORMAL");
        telemetry.addData("Intake decrease (gamepad 1 LEFT)", gamepad1.dpad_left ? "PRESSED" : "Released");
        telemetry.addData("Intake increase (gamepad 1 RIGHT)", gamepad1.dpad_right ? "PRESSED" : "Released");
        telemetry.addData("Turn input", "%.2f", turn);
        telemetry.addData("Drive", "Robot-relative");
        telemetry.addData("Front left power (FLDC)", "%.2f", frontLeft.getPower());
        telemetry.addData("Front right power (FRDC)", "%.2f", frontRight.getPower());
        telemetry.addData("Rear left power (RLDC)", "%.2f", rearLeft.getPower());
        telemetry.addData("Rear right power (RRDC)", "%.2f", rearRight.getPower());
    }

    @Override public void start() {
        teleopStarted = true;
        helperAWasPressed = gamepad2.a;
        helperBWasPressed = gamepad2.b;

        helperReverseXWasPressed = gamepad2.x; // Ignore buttons held during INIT.
        intakeLeftWasPressed = gamepad1.dpad_left;
        intakeRightWasPressed = gamepad1.dpad_right;
        intakeStartWasPressed = gamepad1.left_trigger > SHOOTER_TRIGGER_THRESHOLD; // Ignore X held during INIT.
        feederStartWasPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        // Require new presses after Driver Station START; ignore buttons held in INIT.
        shooterRightWasPressed = gamepad2.dpad_right;
        shooterLeftWasPressed = gamepad2.dpad_left;
    }

    private void updateShooter() {
        // Ignore pairing chords and simultaneous D-pad LEFT+RIGHT; holding a button changes once.
        boolean increasePressed = gamepad2.dpad_right && !shooterRightWasPressed && !gamepad2.dpad_left
                && !gamepad2.start && !gamepad2.back;
        boolean decreasePressed = gamepad2.dpad_left && !shooterLeftWasPressed && !gamepad2.dpad_right
                && !gamepad2.start && !gamepad2.back;
        boolean speedChanged = increasePressed || decreasePressed;

        // Retain selected speed between sequences; INIT resets it to the configured default.
        if (speedChanged) {
            int previousSpeed = selectedShooterSpeedPercent;
            int step = SHOOTER_STEP_PERCENT;
            selectedShooterSpeedPercent = Math.max(0, Math.min(MAX_SHOOTER_SPEED_PERCENT,
                    selectedShooterSpeedPercent + (increasePressed ? step : -step)));
            lastShooterSpeedChange = (increasePressed ? "D-pad RIGHT" : "D-pad LEFT") + " pressed: "
                    + previousSpeed + " -> " + selectedShooterSpeedPercent
                    + "%" + (previousSpeed == selectedShooterSpeedPercent ? " (limit reached)" : "");
        }
        // D-pad RIGHT/LEFT adjust running speed but never start a stopped shooter.
        // Zero speed also stops the feeder to avoid feeding into a stopped wheel.
        if (speedChanged && selectedShooterSpeedPercent <= 0) {
            stopShootingImmediately();
        } else if (speedChanged && shooter.getTargetVelocity() > 0) {
            shooter.runSpeedFraction(selectedShooterSpeedPercent / 100.0);
        }
        double target = shooter.getTargetVelocity();
        double measured = shooter.measuredVelocity();
        telemetry.addData("Pollen target (ticks/s)", "%.0f", target);
        telemetry.addData("Pollen measured (ticks/s)", "%.0f", measured);
        telemetry.addData("Pollen error (ticks/s)", "%.0f", target - measured);
        telemetry.addData("Pollen shooter", target > 0 ? "Running (velocity control)" : "Stopped (coasts)");
        telemetry.addData("Pollen selected speed (% of reference)", "%d%%", selectedShooterSpeedPercent);
        telemetry.addData("Pollen SDK power (not measured RPM)", "%.0f%%", shooter.getShooterPower() * 100);
        telemetry.addData("Pollen gamepad 2 increase (D-pad RIGHT) / decrease (D-pad LEFT)", "%s / %s",
                gamepad2.dpad_right ? "PRESSED" : "Released", gamepad2.dpad_left ? "PRESSED" : "Released");
        telemetry.addData("Pollen last speed change", lastShooterSpeedChange);
        shooterRightWasPressed = gamepad2.dpad_right;
        shooterLeftWasPressed = gamepad2.dpad_left;
    }

    /** Compatibility for calibration subclasses; Nectar is disabled. */
    protected int getSelectedNectarSpeedPercent() { return 0; }
    @Override public void stop() {
        stopShootingImmediately();
        teleopStarted = false;
        if (intake != null) intake.stop();
        if (driveTest != null) driveTest.drive(0, 0, 0);
        // KKN_TEST's inactive color sensor is not initialized or dereferenced here.
    }
    /** Read-only calibration access; also available when the shooter is stopped. */
    protected int getSelectedShooterSpeedPercent() { return selectedShooterSpeedPercent; }
    /** Rescale past the deadband, then apply a direction-preserving response curve. */
    private static double shapeDriveInput(double input) {
        double magnitude = Math.max(0.0,
                (Math.abs(input) - DRIVE_DEADBAND) / (1.0 - DRIVE_DEADBAND));
        return Math.copySign(Math.pow(magnitude, DRIVE_RESPONSE_EXPONENT), input);
    }
}
