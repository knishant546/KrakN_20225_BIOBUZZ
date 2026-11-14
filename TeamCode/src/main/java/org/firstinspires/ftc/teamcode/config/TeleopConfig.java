/* ============================================================================
 * Class: TeleopConfig
 * What it does: Central driver-control speed, power, adjustment, and delay settings.
 * Applications using it: BioBuzzTeleop; MecanumDrive uses the drive settings.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.config;

/** Edit here, rebuild, and install. Percent values are 0..100, power fractions 0..1. */
public final class TeleopConfig {
    private TeleopConfig() { }
    // Velocity percentages of PollenShooter's encoder reference, not motor power.
    public static final int INITIAL_SHOOTER_SPEED_PERCENT = 60;
    public static final int MAX_SHOOTER_SPEED_PERCENT = 85;
    public static final int SHOOTER_STEP_PERCENT = 1;
    // Independent nectar velocity selection; same initial limits as pollen.
    public static final int INITIAL_NECTAR_SPEED_PERCENT = 58;
    public static final int MAX_NECTAR_SPEED_PERCENT = 85;
    public static final int NECTAR_STEP_PERCENT = 1;
    public static final double FEEDER_POWER = -0.50; // Signed DC motor power.
    public static final int INITIAL_INTAKE_POWER_PERCENT = 90; // Initial intake selection; LT retains the selected power.
    public static final int MAX_INTAKE_POWER_PERCENT = 100;
    public static final int INTAKE_STEP_PERCENT = 1;
    public static final int INITIAL_INTAKE_HELPER_POWER_PERCENT = 80;
    public static final int INTAKE_HELPER_STEP_PERCENT = 2;
    public static final double DRIVE_MAX_POWER = 0.50;
    // Normalize only when mixed wheel commands exceed 1; preserve full stick travel.
    public static final double DRIVE_NORMALIZATION_FLOOR = 1.0;
    public static final double DRIVE_DEADBAND = 0.05;
    public static final double DRIVE_RESPONSE_EXPONENT = 2.0; // 1 = linear; 2 = gentler near center.
    public static final double TURN_DEADBAND = 0.05;
    // Maximum wheel power for turning alone; combined driving is normalized.
    public static final double TURN_MAX_POWER = 0.10;
    public static final double ARM_STEP_DEGREES = 5.0;
    // TeleOp Pollen-only feeding: initial timeout is not used for recovery pauses.
    public static final double SHOOTER_START_TIMEOUT_SECONDS = 1.5;
    public static final double SHOOTER_READY_ERROR_FRACTION = 0.05;
    public static final double SHOOTER_READY_MAX_ERROR_TICKS = 200.0;
    public static final double SHOOTER_DROP_FRACTION = 0.10;
    public static final double SHOOTER_DROP_HOLD_SECONDS = 0.10;
    public static final double SHOOTER_RECOVERY_HOLD_SECONDS = 0.20;
    public static final double SHOOTER_START_DELAY_SECONDS = 1.0; // Legacy; BioBuzzTeleop uses speed gate above.
    public static final double SHOOTER_STOP_DELAY_SECONDS = 1.0;
}