/* ============================================================================
 * Class: TeleopConfig
 * What it does: Central driver-control speed, power, adjustment, and delay settings.
 * Applications using it: BioBuzzTeleop; MecanumDrive uses the drive settings.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.config;

/** Edit here, rebuild, and install. Percent values are 0..100, power fractions 0..1. */
public final class TeleopConfig {
    private TeleopConfig() { }
    // Velocity percentages of ShooterBBTeleop's encoder reference, not motor power.
    public static final int INITIAL_SHOOTER_SPEED_PERCENT = 60;
    public static final int MAX_SHOOTER_SPEED_PERCENT = 85;
    public static final int SHOOTER_STEP_PERCENT = 5;
    public static final double FEEDER_POWER = -0.50; // Signed DC motor power.
    public static final int INITIAL_INTAKE_POWER_PERCENT = 90; // INIT and D-pad RIGHT.
    public static final int MAX_INTAKE_POWER_PERCENT = 100;
    public static final int INTAKE_STEP_PERCENT = 10;
    public static final double DRIVE_MAX_POWER = 0.25;
    public static final double DRIVE_NORMALIZATION_FLOOR = 0.25;
    public static final double TURN_DEADBAND = 0.05;
    public static final double MAX_TURN_INPUT = 0.20;
    public static final double ARM_STEP_DEGREES = 5.0;
    public static final double SHOOTER_START_DELAY_SECONDS = 1.0;
    public static final double SHOOTER_STOP_DELAY_SECONDS = 1.0;
}