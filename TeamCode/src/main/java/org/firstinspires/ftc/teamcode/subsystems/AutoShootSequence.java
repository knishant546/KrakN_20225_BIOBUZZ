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
import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

/** Timed autonomous shooting using the same hardware as BioBuzzTeleop. */
public class AutoShootSequence {
    // Shooter fraction now scales encoder velocity; intake/feeder remain power-controlled.
    // Total duration from sequence start, including the initial one-second spin-up.
    private final Intake intake = new Intake();
    private final InputDCShooter feeder = new InputDCShooter();
    private final ShooterBBTeleop shooter = ShooterBBTeleop.getInstance();
    private long startedAt;
    private double shotSpeedFraction;
    private double totalSeconds;
    private boolean active;
    private boolean keepIntakeAfterShot;
    private boolean feeding;
    private boolean initialized;
    private String status = "Not initialized";

    public void init(HardwareMap hardwareMap) {
        intake.init(hardwareMap);
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

    public Command command() {
        return command(SHOOTER_SPEED_FRACTION, DEFAULT_SHOOT_SECONDS);
    }

    /** Duration includes the one-second spin-up; defaults for RED_AutoGarden stay unchanged. */
    public Command command(double speedFraction, double durationSeconds) {
        return command(speedFraction, durationSeconds, false);
    }

    /** Keep intake after normal completion only; interruption stops all motors. */
    public Command command(double speedFraction, double durationSeconds, boolean keepIntake) {
        if (!Double.isFinite(speedFraction) || speedFraction <= 0 || speedFraction > 1
                || !Double.isFinite(durationSeconds) || durationSeconds <= FEEDER_DELAY_SECONDS)
            throw new IllegalArgumentException("Invalid shooting speed fraction or duration");
        return new LambdaCommand("IntakeShootThenPark")
                .setStart(() -> {
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
        startedAt = System.nanoTime();
        active = true;
        intake.start(INTAKE_POWER);
        shooter.runSpeedFraction(shotSpeedFraction);
        status = "Intake + shooter running; feeder waiting";
    }

    private void update() {
        if (!active) return;
        double elapsed = (System.nanoTime() - startedAt) / 1e9;
        // Check completion first so a delayed loop cannot briefly start the feeder after timeout.
        if (elapsed >= totalSeconds) {
            finishShot();
        } else if (!feeding && elapsed >= FEEDER_DELAY_SECONDS) {
            feeder.start();
            feeding = true;
            status = "Intake + shooter + feeder running";
        }
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