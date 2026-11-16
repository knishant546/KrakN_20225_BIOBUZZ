/* ============================================================================
 * Class: BioBuzzTeleop
 * What it does:
 *   Runs two-controller driving, intake/arm controls, and sequenced shooter/feeder operation.
 * Applications using it:
 *   BioBuzzTeleop (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TeleopAppl;
import org.firstinspires.ftc.teamcode.Logging.RunTelemetryLogger;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.config.TeleopConfig.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeArmSingleServo;
import org.firstinspires.ftc.teamcode.subsystems.InputDCShooter;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGateServo;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGateControl;


/** Driver-controlled chassis mode based on KKN_TEST's active mecanum test. */
@TeleOp(name = "BioBuzzTeleop", group = "BioBuzz")
public class BioBuzzTeleop extends OpMode {
    private RunTelemetryLogger runLog = new RunTelemetryLogger();
    // Pollen and feeder initialize stopped during INIT; Nectar is disabled.
    // START spins Pollen/feeder; Gamepad 2 LT/RT open/close the gate.
    private PollenShooter shooter;
    private final ShooterGateServo gate = new ShooterGateServo();
    private final ShooterGateControl gateControl = new ShooterGateControl();
    private boolean gateRequested;
    // Test overrides latch once per press and reset at INIT/START.
    private boolean shooterAWasPressed, feederBWasPressed, shooterManualStop;
    private Boolean feederTestOverride; // null = normal automatic startup; true/false = manual.
    private boolean feederOutput;

    private boolean shooterRightWasPressed, shooterLeftWasPressed;
    // Velocity control uses TeleopConfig defaults; RIGHT/LEFT adjust the selected target.
    private int intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
    private boolean intakeLeftWasPressed, intakeRightWasPressed, intakeStartWasPressed;
    private boolean intakeReverse, intakeDirectionXWasPressed;
    // D-pad RIGHT/LEFT adjust speed by 1 percentage point per press, limited to 0..85%.
    private int selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
    private String lastShooterSpeedChange = "No speed change requested";
    private InputDCShooter inputDCShooter;
    // Trigger depth selects a button press, not shooter velocity.
    private static final double SHOOTER_TRIGGER_THRESHOLD = 0.5;
    private boolean feederStartWasPressed;
    private boolean teleopStarted;
    private boolean pollenReversed, feederReversed, pollenXWasPressed, feederYWasPressed;

    private void stopShootingImmediately() {
        gate.close(); // Close before stopping rotating motors.
        if (inputDCShooter != null) inputDCShooter.stop();
        if (shooter != null) shooter.stopImmediately();
        // Shooting controls never change the independent intake request.
        gateRequested = false;
        gate.close();
        gateControl.stop();
        feederStatus = "Stopped";
    }
    private String feederStatus = "Ready / stopped";
    // Keep below MecanumDrive's 0.3 normalization floor so pure turning stays slower.
    private MecanumDrive driveTest;
    private Intake intake;
    private IntakeArmSingleServo intakeArm;

    private boolean intakeRunning, intakeManualStop;
    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    /** Legacy subclass compatibility; the old intake helper no longer exists. */
    protected boolean isIntakeHelperEnabled() { return false; }

    @Override public void init() {
        runLog = new RunTelemetryLogger();
        telemetry = runLog.attach(telemetry, getClass().getSimpleName());
        teleopStarted = false;
        shooterManualStop = false;
        feederTestOverride = null;
        feederOutput = false;
        pollenReversed = feederReversed = false;
        pollenXWasPressed = gamepad2.x;
        feederYWasPressed = gamepad2.y;
        gate.init(hardwareMap); // CLOSED/DOWN during INIT.
        gateRequested = false;
        shooter = PollenShooter.getInstance(); shooter.initialize(hardwareMap);
        gateRequested = false;
        gate.close();
        gateControl.stop(); // Nectar is disabled: no hardware lookup or commands.
        selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
        lastShooterSpeedChange = "No speed change requested";
        driveTest = new MecanumDrive();
        // false selects the REV hub IMU named "imu"; true resets/configures Pinpoint.
        // Robot-relative drive() below does not use either sensor's heading.
        driveTest.init(hardwareMap, false);
        driveTest.drive(0, 0, 0);
        intake = new Intake();
        intake.init(hardwareMap); // Gate is independent of intake.
        intake.stop();
        intakeRunning = false;
        intakeManualStop = false;
        intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
        intakeReverse = false;
        intakeDirectionXWasPressed = gamepad1.x;
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
        telemetry.addLine("Gamepad 2: LT opens gate; RT closes gate; RIGHT/LEFT adjusts Pollen");
        telemetry.addLine("Intake G1: LT start / RT stop; D-pad RIGHT +1 / LEFT -1 point; X toggles direction. Driving does not start intake.");
        telemetry.addLine("Arm: folded UP; after START: gamepad 1 UP/DOWN presets");
        telemetry.addData("Input shooter motor", "START spins Pollen; feeder starts at speed or 1.5 s; gate waits for LT.");
    }

    @Override public void init_loop() {
        // Track buttons during INIT without starting either motor.
        updateInputDCShooter();
    }

    /** LT/RT now request gate OPEN/CLOSED; motors stay running after RT. */
    private void updateInputDCShooter() {
        double now = System.nanoTime()/1e9;
        // A/B ignore controller-assignment chords. Motor toggles preserve the LT/RT gate request.
        if (teleopStarted && !gamepad2.start && !gamepad2.back) {
            if (gamepad2.a && !shooterAWasPressed) {
                gateRequested = false;
                gate.close();
                gateControl.closeGate();
                shooterManualStop = shooter.getTargetVelocity() > 0;
                if (shooterManualStop) {
                    shooter.stopImmediately();
                    gateControl.stop();
                    // Preserve feeder state: this button controls the shooter only.
                    if (feederTestOverride == null) feederTestOverride = feederOutput;
                } else if (selectedShooterSpeedPercent > 0) {
                    shooter.runSpeedFraction(selectedShooterSpeedPercent / 100.0);
                    gateControl.start(now);
                }
            }
            if (gamepad2.b && !feederBWasPressed) {
                feederTestOverride = !feederOutput;
                gateRequested = false;
                gate.close();
                gateControl.closeGate();
            }
        }
        shooterAWasPressed = gamepad2.a;
        feederBWasPressed = gamepad2.b;
        boolean pressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD && !feederStartWasPressed;
        if (gamepad2.right_trigger > SHOOTER_TRIGGER_THRESHOLD) {
            gateRequested = false;
            gate.close();
            gateControl.closeGate();
        } else if (pressed && teleopStarted && selectedShooterSpeedPercent > 0) gateRequested = true;

        // Close and cancel the request before either direction change.
        if (teleopStarted && gamepad2.x && !pollenXWasPressed) {
            gateRequested = false;
            gate.close();
            pollenReversed = !pollenReversed;
            shooter.setReversed(pollenReversed);
            gateControl.requireRecovery();
        }
        if (teleopStarted && gamepad2.y && !feederYWasPressed) {
            gateRequested = false;
            gate.close();
            gateControl.closeGate();
            feederReversed = !feederReversed;
            inputDCShooter.setPower(feederReversed ? FEEDER_POWER : -FEEDER_POWER);
        }
        pollenXWasPressed = gamepad2.x;
        feederYWasPressed = gamepad2.y;
        feederStartWasPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        if (teleopStarted) gateControl.update(now, shooter.getTargetVelocity(),
                shooter.measuredVelocity(), gateRequested && !Boolean.FALSE.equals(feederTestOverride), 0.0);
        feederOutput = teleopStarted && (feederTestOverride != null
                ? feederTestOverride : gateControl.isFeederRunning());
        if (feederOutput) inputDCShooter.start(); else inputDCShooter.stop();
        if (gateControl.isGateOpen()) gate.open(); else gate.close();
        feederStatus = feederOutput ? "Running (gate controls balls)" : "Waiting / stopped";
        telemetry.addData("Test shooter (G2 A)", shooter.getTargetVelocity() > 0 ? "ON" : "OFF");
        telemetry.addData("Test feeder (G2 B)", (feederOutput ? "ON" : "OFF")
                + (feederTestOverride == null ? " / automatic" : " / manual override"));
        telemetry.addData("Pollen direction (G2 X)", pollenReversed ? "REVERSED" : "NORMAL");
        telemetry.addData("Feeder direction (G2 Y)", feederReversed ? "REVERSED (-power)" : "NORMAL (+power)");
        telemetry.addData("Shooter gate", gate.isOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Shooter gate requested", gateRequested ? "OPEN" : "CLOSED");
        telemetry.addData("Shooter gate commanded degrees", gate.getCommandedDegrees());
    }
    @Override public void loop() {
        runLog.updateToggle(gamepad1.back);
        runLog.gamepads(gamepad1, gamepad2);
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
        // Gamepad 1 X toggles DC intake direction once per press, preserving power/start state.
        if (gamepad1.x && !intakeDirectionXWasPressed) intakeReverse = !intakeReverse;
        intakeDirectionXWasPressed = gamepad1.x;
        telemetry.addData("Intake direction (G1 X)", intakeReverse ? "REVERSE (+power)" : "NORMAL (-power)");
        // Intake remains independent of gate and shooter readiness.
        boolean intakeOutput = intakeRunning && !intakeManualStop && intakePowerPercent > 0;
        if (intakeOutput) intake.start(intakePowerPercent / 100.0, intakeReverse, false);
        else intake.stop();
        telemetry.addData("Input shooter motor", feederStatus);
        telemetry.addData("Shooting sequence", gateControl.getState());
        telemetry.addData("Intake actual output", intakeOutput ? "Running (independent of shooter)" : "Stopped");
        telemetry.addData("Pollen ready tolerance (ticks/s)", "%.0f",
                Math.min(SHOOTER_READY_MAX_ERROR_TICKS, shooter.getTargetVelocity() * SHOOTER_READY_ERROR_FRACTION));
        telemetry.addData("Nectar", "DISABLED (no hardware initialized)");
        telemetry.addData("Intake", intakeManualStop ? "Manual STOP (LT to restart)"
                : intakePowerPercent == 0 ? "Stopped (0% selected power)" : intakeRunning ? "Running" : "Waiting for Gamepad 1 LT");
        telemetry.addData("Intake selected power", "%d%% (D-pad step: %d points)", intakePowerPercent, INTAKE_STEP_PERCENT);
        telemetry.addData("Intake decrease (gamepad 1 LEFT)", gamepad1.dpad_left ? "PRESSED" : "Released");
        telemetry.addData("Intake increase (gamepad 1 RIGHT)", gamepad1.dpad_right ? "PRESSED" : "Released");
        telemetry.addData("Turn input", "%.2f", turn);
        telemetry.addData("Drive", "Robot-relative");
        telemetry.addData("Front left power (FLDC)", "%.2f", frontLeft.getPower());
        telemetry.addData("Front right power (FRDC)", "%.2f", frontRight.getPower());
        telemetry.addData("Rear left power (RLDC)", "%.2f", rearLeft.getPower());
        telemetry.addData("Rear right power (RRDC)", "%.2f", rearRight.getPower());
        runLog.sample(hardwareMap, telemetry);
    }

    @Override public void start() {
        runLog.beginRun(gamepad1.back);
        runLog.event("lifecycle", "START");
        pollenXWasPressed = gamepad2.x;
        feederYWasPressed = gamepad2.y;
        intakeDirectionXWasPressed = gamepad1.x; // Require a fresh press after START.
        teleopStarted = true;
        shooterAWasPressed = gamepad2.a;
        feederBWasPressed = gamepad2.b;
        shooterManualStop = false;
        feederTestOverride = null;
        gateRequested = false;
        gate.close();
        if (selectedShooterSpeedPercent > 0) {
            shooter.runSpeedFraction(selectedShooterSpeedPercent / 100.0);
            gateControl.start(System.nanoTime()/1e9);
        }

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
        // Selected zero stops motors/closes gate; increasing again restarts the startup check.
        // Zero speed also stops the feeder to avoid feeding into a stopped wheel.
        if (speedChanged && selectedShooterSpeedPercent <= 0) {
            stopShootingImmediately();
        } else if (speedChanged && teleopStarted && !shooterManualStop) {
            if (shooter.getTargetVelocity() <= 0) gateControl.start(System.nanoTime()/1e9);
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
        shooterManualStop = false;
        feederTestOverride = null;
        feederOutput = false;
        pollenReversed = feederReversed = false;
        pollenXWasPressed = gamepad2.x;
        feederYWasPressed = gamepad2.y;
        if (intake != null) intake.stop();
        if (driveTest != null) driveTest.drive(0, 0, 0);
        // KKN_TEST's inactive color sensor is not initialized or dereferenced here.
        runLog.close();
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
