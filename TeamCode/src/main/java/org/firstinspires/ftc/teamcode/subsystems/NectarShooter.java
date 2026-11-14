/* ============================================================================
 * Class: NectarShooter
 * What it does: Controls the nectar motor with REV Hub encoder velocity PIDF.
 * Applications using it: BioBuzzTeleop and its ShooterShotCalibration subclass.
 * Autonomous uses PollenShooter only. Parameters initially match PollenShooter;
 * tune this motor independently after verifying its encoder and direction.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/** Encoder velocity control using the REV Hub velocity PIDF and configured motor type. */
public class NectarShooter implements Subsystem {
    public static final double MAX_VELOCITY_TICKS_PER_SECOND = 2600;
    private static final NectarShooter INSTANCE = new NectarShooter();
    private DcMotorEx shooterMotor;
    private double targetVelocity = 0;
    private double speedFactor = 1;

    public static NectarShooter getInstance() { return INSTANCE; }
    private NectarShooter() { }

    @Override public void initialize() { initialize(ActiveOpMode.hardwareMap()); }

    /** Ordinary FTC OpModes may initialize directly without a NextFTC scheduler. */
    public void initialize(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "nectarShooter");
        shooterMotor.setPower(0);
        // Reverse physical rotation while keeping positive velocity commands.
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        speedFactor = 1;
        stopImmediately(); // INIT always leaves the shooter stopped, with target zero.
    }

    /** Fraction of the configured velocity reference, not a motor power percentage. */
    public void runSpeedFraction(double fraction) {
        if (!Double.isFinite(fraction) || fraction < 0 || fraction > 1)
            throw new IllegalArgumentException("Shooter speed fraction must be 0..1");
        runVelocity(fraction * MAX_VELOCITY_TICKS_PER_SECOND);
    }

    public void runVelocity(double ticksPerSecond) {
        if (!Double.isFinite(ticksPerSecond) || ticksPerSecond < 0
                || ticksPerSecond > MAX_VELOCITY_TICKS_PER_SECOND)
            throw new IllegalArgumentException("Shooter velocity must be 0..2600 ticks/s");
        requireInitialized();
        if (ticksPerSecond == 0) { stopImmediately(); return; }
        shooterMotor.setVelocity(ticksPerSecond);
        targetVelocity = ticksPerSecond;
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double measuredVelocity() {
        requireInitialized();
        return shooterMotor.getVelocity();
    }
    /** Immediate shutdown; safe before initialization and independent of scheduling. */
    public void stopImmediately() {
        targetVelocity = 0;
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0);
            shooterMotor.setPower(0);
        }
    }

    public float getShooterPower() {
        requireInitialized();
        return (float) shooterMotor.getPower();
    }

    /** Scales subsequent preset/start commands only; does not change a running target. */
    public void setSpeedFactor(double factor) {
        if (!Double.isFinite(factor) || factor < 0 || factor > 1) {
            throw new IllegalArgumentException("Shooter speed factor must be 0..1");
        }
        speedFactor = factor;
    }
    public double getSpeedFactor() { return speedFactor; }

    public Command startShooter() {
        return new InstantCommand(() -> runSpeedFraction(speedFactor))
                .requires(this);
    }
    // Absolute presets, not increments. Preserve the original 85% and 70% settings.
    public final Command select85PercentSpeed = new InstantCommand(() ->
            runSpeedFraction(speedFactor * 0.85)).requires(this);
    public final Command select70PercentSpeed = new InstantCommand(() ->
            runSpeedFraction(speedFactor * 0.70)).requires(this);

    public Command stopShooter() {
        return new InstantCommand(this::stopImmediately).requires(this);
    }
    private void requireInitialized() {
        if (shooterMotor == null) throw new IllegalStateException("Initialize NectarShooter before use");
    }
}