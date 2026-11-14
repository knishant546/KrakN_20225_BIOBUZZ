/* ============================================================================
 * Class: AutoShootSequence
 * What it does:
 *   Coordinates autonomous intake, encoder-controlled shooter, delayed feeder, and collection commands.
 * Applications using it:
 *   BLUE_AutoFlower
 *   BLUE_AutoGarden
 *   RED_AutoFlower
 *   RED_AutoGarden
 *   Includes indirect/source dependencies; optional features may be inactive.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.config.TeleopConfig;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

/** Timed autonomous shooting using the same hardware as BioBuzzTeleop. */
public class AutoShootSequence {
    // Shooter fraction now scales encoder velocity; intake/feeder remain power-controlled.
    // Total duration from sequence start, including the configured shooter-to-intake delay.
    private final Intake intake = new Intake();
    private final InputDCShooter feeder = new InputDCShooter();
    private final PollenShooter shooter = PollenShooter.getInstance();
    private long startedAt;
    private double shotSpeedFraction;
    private double totalSeconds;
    private boolean active;
    private boolean keepIntakeAfterShot;
    private boolean feeding;
    private boolean initialized;
    private boolean speedGatedFeeding;
    private boolean recovering;
    private boolean allowStartupTimeout = true;
    private double lowSince = Double.NaN, readySince = Double.NaN;
    private String status = "Not initialized";

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    /** Garden passes false to suppress helper hardware and movement during collection too. */
    public void init(HardwareMap hardwareMap, boolean enableHelper) {
        intake.init(hardwareMap, enableHelper, true); // Helper, when enabled, runs opposite TeleOp.
        intake.stop();
        shooter.initialize(hardwareMap);
        feeder.init(hardwareMap);
        feeder.setPower(FEEDER_POWER); // Auto has its own power setting.
        initialized = true;
        stop();
        status = "Ready / stopped";
    }

    /** Collect without running shooter/feeder; stop intake on completion or interruption. */
    public Command collectCommand(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0)
            throw new IllegalArgumentException("Collection duration must be positive");
        final long[] collectionStarted = new long[1];
        return new LambdaCommand("Collect garden pollen")
                .setStart(() -> {
                    if (!initialized) throw new IllegalStateException("Initialize shooting sequence first");
                    stop();
                    collectionStarted[0] = System.nanoTime();
                    intake.start(INTAKE_POWER);
                    status = "Collecting: intake only";
                })
                .setIsDone(() -> (System.nanoTime() - collectionStarted[0]) / 1e9 >= seconds)
                .setStop(interrupted -> stop())
                .requires(this);
    }
    /** Intake-only collection that continues until stop() or a shooting command takes over. */
    public void startCollecting() {
        if (!initialized) throw new IllegalStateException("Initialize shooting sequence first");
        stop();
        intake.start(INTAKE_POWER);
        status = "Collecting: intake only";
    }

    /** Flower collection/return: DC intake only; shooter, feeder and helper stay off. */
    public void startCollectingDcOnly() {
        if (!initialized) throw new IllegalStateException("Initialize shooting sequence first");
        stop();
        intake.start(INTAKE_POWER, false, false);
        status = "Collecting: DC intake only; shooter/helper/feeder OFF";
    }

    /** Full Garden/NoFlwr START: run holding intake and Pollen until parking/STOP. */
    public void startContinuous(double fraction) {
        if (!initialized) throw new IllegalStateException("Initialize shooting sequence first");
        stopFeeder();
        feeder.setPower(-TeleopConfig.FEEDER_POWER);
        shooter.runSpeedFraction(fraction);
        intake.start(INTAKE_POWER, false, false);
        status = "Travel/collection: Pollen + intake running; feeder OFF";
    }

    /** End shot permission without disturbing the continuous intake/shooter. */
    public void stopFeeder() {
        active = false;
        feeding = recovering = false;
        lowSince = readySince = Double.NaN;
        feeder.stop();
        status = "Pollen + intake holding; feeder OFF";
    }

    /** Garden shot window: actual pose permission AND speed readiness; no timed fallback.
     * Cancellation ends only feeding. The owning OpMode stops everything at STOP/30 s.
     */
    public Command continuousShotCommand(double seconds, BooleanSupplier atPosition) {
        if (!Double.isFinite(seconds) || seconds <= 0 || atPosition == null)
            throw new IllegalArgumentException("Invalid shot window");
        return new LambdaCommand("Garden position and speed gated shot")
                .setStart(() -> {
                    stopFeeder();
                    allowStartupTimeout = false;
                    startedAt = System.nanoTime();
                    active = true;
                    status = "Shot window: waiting for position and Pollen speed";
                })
                .setUpdate(() -> {
                    double elapsed = (System.nanoTime() - startedAt) / 1e9;
                    if (elapsed >= seconds) { stopFeeder(); return; }
                    if (!atPosition.getAsBoolean()) {
                        feeder.stop();
                        feeding = false;
                        recovering = true;
                        readySince = lowSince = Double.NaN;
                        status = "Feeder blocked: shooting position not confirmed";
                        return;
                    }
                    updateSpeedGatedFeeding(elapsed);
                })
                .setIsDone(() -> !active || (System.nanoTime() - startedAt) / 1e9 >= seconds)
                .setStop(interrupted -> stopFeeder())
                .requires(this);
    }

    public Command command() {
        return command(SHOOTER_SPEED_FRACTION, DEFAULT_SHOOT_SECONDS);
    }

    /** Duration includes the configured shooter-to-intake delay. */
    public Command command(double speedFraction, double durationSeconds) {
        return command(speedFraction, durationSeconds, false);
    }

    /** Keep intake after normal completion only; interruption stops all motors. */
    public Command command(double speedFraction, double durationSeconds, boolean keepIntake) {
        return buildCommand(speedFraction, durationSeconds, keepIntake, false);
    }

    /** Flower entry point retained for existing callers. */
    public Command flowerCommand(double speedFraction, double durationSeconds) {
        return speedGatedCommand(speedFraction, durationSeconds, false);
    }

    /** TeleOp-style speed readiness/recovery for Flower and full Garden routines.
     * Total shot duration includes spin-up/recovery. Keep intake only after normal
     * completion when Garden requests it for the following collection trip.
     */
    public Command speedGatedCommand(double speedFraction, double durationSeconds, boolean keepIntake) {
        return buildCommand(speedFraction, durationSeconds, keepIntake, true);
    }

    private Command buildCommand(double speedFraction, double durationSeconds,
                                 boolean keepIntake, boolean speedGated) {
        if (!Double.isFinite(speedFraction) || speedFraction <= 0 || speedFraction > 1
                || !Double.isFinite(durationSeconds) || durationSeconds <= SHOOTER_TO_INTAKE_DELAY_SECONDS)
            throw new IllegalArgumentException("Invalid shooting speed fraction or duration");
        return new LambdaCommand("IntakeShootThenPark")
                .setStart(() -> {
                    allowStartupTimeout = true;
                    speedGatedFeeding = speedGated;
                    keepIntakeAfterShot = keepIntake;
                    shotSpeedFraction = speedFraction;
                    totalSeconds = durationSeconds;
                    start();
                })
                .setUpdate(this::update)
                .setIsDone(() -> !active)
                .setStop(interrupted -> { if (interrupted || !keepIntake) stop(); else finishShot(); })
                .requires(this);
    }

    private void start() {
        if (!initialized) throw new IllegalStateException("Initialize shooting sequence first");
        feeder.stop();
        feeding = false;
        recovering = false;
        lowSince = readySince = Double.NaN;
        // Speed-gated routines use corrected TeleOp feeder power; legacy commands retain Auto power.
        feeder.setPower(speedGatedFeeding ? -TeleopConfig.FEEDER_POWER : FEEDER_POWER);
        startedAt = System.nanoTime();
        active = true;
        // Active autonomous shots start intake and Pollen together; only feeder waits.
        intake.stop();
        shooter.runSpeedFraction(shotSpeedFraction);
        if (speedGatedFeeding) intake.start(INTAKE_POWER, false, false);
        status = speedGatedFeeding ? "Pollen + intake running: feeder waiting for speed or 1.5 s" : "Shooter spinning up; intake waiting; helper + feeder OFF";
    }

    private void update() {
        if (!active) return;
        double elapsed = (System.nanoTime() - startedAt) / 1e9;
        // Check completion first so a delayed loop cannot briefly start the feeder after timeout.
        if (elapsed >= totalSeconds) {
            finishShot();
        } else if (speedGatedFeeding) {
            updateSpeedGatedFeeding(elapsed);
        } else if (!feeding && elapsed >= SHOOTER_TO_INTAKE_DELAY_SECONDS) {
            intake.start(INTAKE_POWER, false, false); // Shooting: DC intake only; helper and feeder stay stopped.
            feeding = true;
            status = "Intake DC + shooter running; helper + feeder OFF";
        }
    }

    /** Only the feeder follows encoder recovery; intake continues holding collected balls. */
    private void updateSpeedGatedFeeding(double elapsed) {
        double target = shooter.getTargetVelocity();
        double measured = shooter.measuredVelocity();
        if (!Double.isFinite(target) || target <= 0) { stop(); return; }
        double tolerance = Math.min(TeleopConfig.SHOOTER_READY_MAX_ERROR_TICKS,
                target * TeleopConfig.SHOOTER_READY_ERROR_FRACTION);
        boolean ready = Double.isFinite(measured) && Math.abs(target - measured) <= tolerance;
        if (feeding) {
            boolean slow = !Double.isFinite(measured)
                    || measured < target * (1 - TeleopConfig.SHOOTER_DROP_FRACTION);
            if (!slow) lowSince = Double.NaN;
            else {
                if (Double.isNaN(lowSince)) lowSince = elapsed;
                if (elapsed - lowSince >= TeleopConfig.SHOOTER_DROP_HOLD_SECONDS) {
                    feeder.stop();
                    feeding = false;
                    recovering = true;
                    readySince = Double.NaN;
                    status = "Pollen recovering: feeder PAUSED; intake running";
                }
            }
        } else if (recovering) {
            if (!ready) readySince = Double.NaN;
            else {
                if (Double.isNaN(readySince)) readySince = elapsed;
                if (elapsed - readySince >= TeleopConfig.SHOOTER_RECOVERY_HOLD_SECONDS)
                    startGatedFeeding("Pollen recovered: feeder resumed; intake running");
            }
        } else if (ready || (allowStartupTimeout && elapsed >= TeleopConfig.SHOOTER_START_TIMEOUT_SECONDS)) {
            // Initial fallback is intentional; it never bypasses a later recovery pause.
            startGatedFeeding(ready ? "Pollen ready: intake + feeder running"
                    : "Startup timeout: intake + feeder running WITHOUT speed confirmation");
        }
    }

    private void startGatedFeeding(String message) {
        // Intake already runs from shot start; do not alter it on feeder resume.
        feeder.start();
        feeding = true;
        recovering = false;
        lowSince = readySince = Double.NaN;
        status = message;
    }

    private void finishShot() {
        if (!keepIntakeAfterShot) { stop(); return; }
        active = false;
        feeding = false;
        feeder.stop();
        shooter.stopImmediately();
        // Preserve continuous intake between the second shot and flower collection.
        intake.start(INTAKE_POWER);
        status = "Shot complete: intake running, shooter and feeder stopped";
    }
    /** Stop feeder first, then intake and shooter; also used on OpMode STOP/interruption. */
    public void stop() {
        active = false;
        feeding = false;
        feeder.stop();
        if (initialized) intake.stop();
        shooter.stopImmediately();
        status = "Stopped";
    }

    public String getStatus() { return status; }
}