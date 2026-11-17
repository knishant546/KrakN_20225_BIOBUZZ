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
import org.firstinspires.ftc.teamcode.subsystems.ShooterBBTeleop;

/** Driver-controlled chassis mode based on KKN_TEST's active mecanum test. */
@TeleOp(name = "BioBuzzTeleop", group = "BioBuzz")
public class BioBuzzTeleop extends OpMode {
    private RunTelemetryLogger runLog = new RunTelemetryLogger();
    // Both motors initialize stopped during INIT.
    // Gamepad 2 LT sequences the shooter and feeder after Driver Station START.
    private ShooterBBTeleop shooter;
    private boolean shooterAWasPressed, shooterBWasPressed;
    // Velocity control: start at 60% of 2600 ticks/s, with A/B limited to 0..85%.
    private int intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
    private boolean intakeBWasPressed, intakeAWasPressed;
    private boolean intakeReverse, intakeReverseXWasPressed;
    // A/B adjust speed by 1 percentage point per press, limited to 0..85%.
    private int selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
    private String lastShooterSpeedChange = "No speed change requested";
    private InputDCShooter inputDCShooter;
    // Trigger depth selects a button press, not shooter velocity.
    private static final double SHOOTER_TRIGGER_THRESHOLD = 0.5;
    private boolean feederStartWasPressed;
    private boolean feederManualStop, feederRunning, feederRightWasPressed;
    private enum FeedSequence { IDLE, SPINNING_UP, FEEDING, SPINNING_DOWN }
    private FeedSequence feedSequence = FeedSequence.IDLE;
    private long sequenceDeadline;
    private boolean teleopStarted;

    private void stopShootingImmediately() {
        if (inputDCShooter != null) inputDCShooter.stop();
        if (shooter != null) shooter.stopImmediately();
        feedSequence = FeedSequence.IDLE;
        feederRunning = false;
        feederManualStop = true;
        feederStatus = "Stopped";
    }
    private String feederStatus = "Ready / stopped";
    // Keep below MecanumDrive's 0.3 normalization floor so pure turning stays slower.
    private double minimumDriveVoltage = Double.POSITIVE_INFINITY;
    private MecanumDrive driveTest;
    private Intake intake;
    private IntakeArmSingleServo intakeArm;
    private boolean intakeRunning, intakeManualStop;
    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    @Override public void init() {
        runLog = new RunTelemetryLogger();
        telemetry = runLog.attach(telemetry, getClass().getSimpleName());
        teleopStarted = false;
        feedSequence = FeedSequence.IDLE;
        feederRunning = false;
        feederManualStop = true;
        shooter = ShooterBBTeleop.getInstance(); shooter.initialize(hardwareMap);
        selectedShooterSpeedPercent = INITIAL_SHOOTER_SPEED_PERCENT;
        lastShooterSpeedChange = "No speed change requested";
        driveTest = new MecanumDrive();
        // false selects the REV hub IMU named "imu"; true resets/configures Pinpoint.
        // Robot-relative drive() below does not use either sensor's heading.
        driveTest.init(hardwareMap, false);
        driveTest.setSlewEnabled(DRIVE_SLEW_ENABLED);
        minimumDriveVoltage = Double.POSITIVE_INFINITY;
        driveTest.drive(0, 0, 0);
        intake = new Intake();
        intake.init(hardwareMap); // Same intake hardware as KKN_TEST; running power is set below.
        intake.stop();
        intakeRunning = false;
        intakeManualStop = false;
        intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
        intakeReverse = false;
        intakeReverseXWasPressed = gamepad2.x;
        intakeBWasPressed = gamepad1.b;
        intakeAWasPressed = gamepad1.a;
        intakeArm = new IntakeArmSingleServo();
        intakeArm.init(hardwareMap); // Commands the configured UP_DEGREES during INIT.
        inputDCShooter = new InputDCShooter(); inputDCShooter.init(hardwareMap);
        inputDCShooter.setPower(FEEDER_POWER);
        feederStatus = "Ready / stopped";
        feederStartWasPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        // MecanumDrive owns motor directions and encoder modes, matching KKN_TEST.
        frontLeft = hardwareMap.get(DcMotor.class, "FLDC");
        frontRight = hardwareMap.get(DcMotor.class, "FRDC");
        rearLeft = hardwareMap.get(DcMotor.class, "RLDC");
        rearRight = hardwareMap.get(DcMotor.class, "RRDC");
        telemetry.addLine("Ready: left stick drive/strafe, right stick X turn");
        telemetry.addLine("Shooter after Driver Station START: gamepad LT sequence, RT stop sequence, A +1%, B -1% (gamepad 2)");
        telemetry.addLine("Intake: auto-start on drive; LT start/reset to 90% / RT stop (latched); Gamepad 1 B -1 / A +1 percentage points; Gamepad 2 X reverses intake");
        telemetry.addLine("Arm: folded UP; after START: gamepad 1 UP/DOWN presets; gamepad 2 has no arm controls");
        telemetry.addData("Input shooter motor", "Ready / stopped; Gamepad 2 LT: shooter then feeder; RT: feeder then shooter");
    }

    @Override public void init_loop() {
        // Track buttons during INIT without starting either motor.
        updateInputDCShooter();
    }

    private void updateInputDCShooter() {
        long now = System.nanoTime();
        boolean rt = gamepad2.right_trigger > SHOOTER_TRIGGER_THRESHOLD;
        boolean startPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD && !feederStartWasPressed
                && !gamepad2.a && !gamepad2.b;
        boolean manualStart = gamepad2.dpad_right && !feederRightWasPressed;
        if (rt) {
            feederRunning = false;
            feederManualStop = true;
            if (feedSequence == FeedSequence.SPINNING_UP || feedSequence == FeedSequence.FEEDING) {
                feedSequence = FeedSequence.SPINNING_DOWN;
                sequenceDeadline = now + (long)(SHOOTER_STOP_DELAY_SECONDS*1e9);
            }
        } else if (startPressed && teleopStarted && selectedShooterSpeedPercent > 0
                && feedSequence == FeedSequence.IDLE) {
            shooter.runSpeedFraction(selectedShooterSpeedPercent/100.0);
            feederRunning = false;
            feederManualStop = false;
            feedSequence = FeedSequence.SPINNING_UP;
            sequenceDeadline = now + (long)(SHOOTER_START_DELAY_SECONDS*1e9);
        }
        double target = shooter.getTargetVelocity();
        double error = Math.abs(target-shooter.measuredVelocity());
        boolean ready = target > 0 && Double.isFinite(error)
                && error <= Math.min(SHOOTER_READY_MAX_ERROR_TICKS, target*SHOOTER_READY_ERROR_FRACTION);
        if (feedSequence == FeedSequence.SPINNING_UP && !rt && target > 0
                && (ready || now >= sequenceDeadline)) {
            if (!feederManualStop) feederRunning = true;
            feedSequence = FeedSequence.FEEDING;
        } else if (feedSequence == FeedSequence.SPINNING_DOWN && now >= sequenceDeadline) {
            stopShootingImmediately();
        }
        // Manual LEFT overrides auto-start in this same loop and remains latched.
        // RIGHT deliberately permits feeder-only testing; does not start the shooter.
        if (teleopStarted && gamepad2.dpad_left) {
            feederManualStop = true; feederRunning = false;
        } else if (teleopStarted && manualStart && !rt && feedSequence != FeedSequence.SPINNING_DOWN) {
            feederManualStop = false; feederRunning = true;
        }
        if (!teleopStarted) feederRunning = false;
        if (feederRunning) inputDCShooter.start(); else inputDCShooter.stop();
        feederStatus = feederRunning ? "Running" : feederManualStop ? "Manual STOP / D-pad RIGHT to start" : "Waiting for shooter speed or 1.5 s";
        feederRightWasPressed = gamepad2.dpad_right;
        feederStartWasPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
        telemetry.addData("Input shooter motor", feederStatus);
        telemetry.addData("Shooting sequence", feedSequence);
        telemetry.addData("Shooter ready for feeder", ready);
        telemetry.addData("Feeder manual controls", "G2 LEFT stop / RIGHT start; RT overrides");
        if (feedSequence == FeedSequence.SPINNING_UP || feedSequence == FeedSequence.SPINNING_DOWN)
            telemetry.addData("Sequence delay remaining (s)", "%.2f", Math.max(0,(sequenceDeadline-now)/1e9));
    }
    @Override public void loop() {
        runLog.updateToggle(gamepad1.back);
        runLog.gamepads(gamepad1, gamepad2);
        updateShooter();
        updateInputDCShooter();
        // FTC calls loop() only after START. Ignore all multi-direction GP1 D-pad input.
        // No button means retain the last arm position; motor controls remain separate.
        int gp1DpadCount = (gamepad1.dpad_up ? 1 : 0) + (gamepad1.dpad_down ? 1 : 0)
                + (gamepad1.dpad_left ? 1 : 0) + (gamepad1.dpad_right ? 1 : 0);
        if (gp1DpadCount == 1 && gamepad1.dpad_up) intakeArm.liftUp();
        else if (gp1DpadCount == 1 && gamepad1.dpad_down) intakeArm.liftDown();
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
        double turn = Math.copySign(Math.min(TURN_MAX_POWER, DRIVE_MAX_POWER) / DRIVE_MAX_POWER * magnitude * magnitude, stickTurn);
        driveTest.drive(shapeDriveInput(-gamepad1.left_stick_y), shapeDriveInput(gamepad1.left_stick_x), turn);
        telemetry.addData("Drive ramp", "%s; %.0f ms; cap %.0f%%", DRIVE_SLEW_ENABLED ? "ON" : "OFF",
                DRIVE_RAMP_SECONDS*1000, DRIVE_MAX_POWER*100);
        telemetry.addData("Wheel requested FL/RL/FR/RR", java.util.Arrays.toString(driveTest.getRequestedPowers()));
        telemetry.addData("Wheel ramped FL/RL/FR/RR", java.util.Arrays.toString(driveTest.getOutputPowers()));
        double voltage = Double.POSITIVE_INFINITY;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            double value = sensor.getVoltage();
            if (value > 0 && Double.isFinite(value)) voltage = Math.min(voltage, value);
        }
        if (Double.isFinite(voltage)) {
            minimumDriveVoltage = Math.min(minimumDriveVoltage, voltage);
            telemetry.addData("Battery current / minimum V", "%.2f / %.2f", voltage, minimumDriveVoltage);
        }
        // Start on any driving axis, ignoring small stick drift. Right-stick Y is unused.
        boolean driving = Math.abs(gamepad1.left_stick_y) > TURN_DEADBAND
                || Math.abs(gamepad1.left_stick_x) > TURN_DEADBAND
                || Math.abs(stickTurn) > TURN_DEADBAND;
        // Manual stop takes priority, even while driving or if both buttons are pressed.
        // Keep it latched so joystick input cannot undo the driver's stop command.
        if (gamepad1.right_trigger > SHOOTER_TRIGGER_THRESHOLD) {
            intakeManualStop = true;
            intakeRunning = false;
        } else if (gamepad1.left_trigger > SHOOTER_TRIGGER_THRESHOLD) {
            // LT always restores the default 90% power, including after X reaches zero.
            intakePowerPercent = INITIAL_INTAKE_POWER_PERCENT;
            intakeManualStop = false;
            intakeRunning = true; // Can also collect while the chassis is stationary.
        } else if (driving && !intakeManualStop) {
            intakeRunning = true;
        }
        // Releasing the sticks does not stop intake; use RT or OpMode STOP.
        // B/A adjust once per press; LT reset wins and simultaneous A+B is ignored.
        if (gamepad1.b && !intakeBWasPressed && !gamepad1.a && gamepad1.left_trigger <= SHOOTER_TRIGGER_THRESHOLD && !gamepad1.start && !gamepad1.back)
            intakePowerPercent = Math.max(0, intakePowerPercent - INTAKE_STEP_PERCENT);
        else if (gamepad1.a && !intakeAWasPressed && !gamepad1.b && gamepad1.left_trigger <= SHOOTER_TRIGGER_THRESHOLD && !gamepad1.start && !gamepad1.back)
            intakePowerPercent = Math.min(MAX_INTAKE_POWER_PERCENT, intakePowerPercent + INTAKE_STEP_PERCENT);
        intakeBWasPressed = gamepad1.b;
        intakeAWasPressed = gamepad1.a;
        // Gamepad 2 X toggles direction once per press; it does not start a stopped intake.
        if (gamepad2.x && !intakeReverseXWasPressed) intakeReverse = !intakeReverse;
        intakeReverseXWasPressed = gamepad2.x;
        if (intakeRunning) intake.start(intakePowerPercent / 100.0, intakeReverse);
        else intake.stop();
        telemetry.addData("Intake", intakeManualStop ? "Manual STOP (LT to restart)"
                : intakePowerPercent == 0 ? "Stopped (0% selected power)" : intakeRunning ? "Running" : "Waiting for drive or LT");
        telemetry.addData("Intake selected power", "%d%% (A/B step: %d points)", intakePowerPercent, INTAKE_STEP_PERCENT);
        telemetry.addData("Intake direction (gamepad 2 X toggles)", intakeReverse ? "REVERSE" : "NORMAL" );
        telemetry.addData("Intake decrease (gamepad 1 B)", gamepad1.b ? "PRESSED" : "Released");
        telemetry.addData("Intake increase (gamepad 1 A)", gamepad1.a ? "PRESSED" : "Released");
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
        teleopStarted = true;
        feederRightWasPressed = gamepad2.dpad_right;
        intakeReverseXWasPressed = gamepad2.x; // Ignore buttons held during INIT.
        intakeBWasPressed = gamepad1.b;
        intakeAWasPressed = gamepad1.a; // Ignore X held during INIT.
        feederStartWasPressed = gamepad2.left_trigger > SHOOTER_TRIGGER_THRESHOLD;
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
    
        runLog.close();
    }
    private static double shapeDriveInput(double input) {
        double value = Math.max(0, (Math.abs(input)-DRIVE_DEADBAND)/(1-DRIVE_DEADBAND));
        return Math.copySign(Math.pow(value, DRIVE_RESPONSE_EXPONENT), input);
    }
}