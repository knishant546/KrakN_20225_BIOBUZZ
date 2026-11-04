/* ============================================================================
 * Class: IntakeArmSingleServo
 * What it does:
 *   Commands the positional intake-arm servo to up, down, or intermediate angles.
 * Applications using it:
 *   BioBuzzTeleop
 *   BLUE_AutoFlower
 *   BLUE_AutoGarden
 *   RED_AutoFlower
 *   RED_AutoGarden
 *   Includes indirect/source dependencies; optional features may be inactive.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Single-servo three-position mechanism for the flower intake arm.
 * Intended for goBILDA 2000-0025-0002 in POSITIONAL mode, with 300-degree travel.
 * Direct methods work in an ordinary FTC OpMode without a NextFTC scheduler.
 * No gamepad bindings or automatic cycling are included.
 */
public class IntakeArmSingleServo {
    public static final String HARDWARE_NAME = "intakeServo";
    private static final double SERVO_RANGE_DEGREES = 300.0;
    public static final double DOWN_DEGREES = 150.0;
    // Tunable intermediate arm command angle.
    public static final double INTERMEDIATE_DEGREE = 135.0;
    public static final double UP_DEGREES = 30.0;

    // Servo command angles for the current horn mounting, not measured arm angles.
    // Smaller values raise the arm: UP=30, intermediate=135, DOWN=90.
    // Adjust the offset/direction only when recalibrating the physical linkage.
    private static final double FLOOR_OFFSET_DEGREES = 0.0;
    private static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;
    private ServoImplEx servo;
    private double commandedArmDegrees = Double.NaN;

    /** Maps the servo and immediately commands UP_DEGREES during INIT. */
    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, HARDWARE_NAME);
        servo.resetDeviceConfigurationForOpMode();
        servo.setDirection(DIRECTION);
        // Specified 500..2500 us gives 300 degrees; the SDK default may be narrower.
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        liftUp();
    }

    /** Fold upward to the configured servo angle. */
    public void liftUp() { setArmPosition(UP_DEGREES); }

    /** Move to the configured DOWN_DEGREES position. */
    public void liftDown() { setArmPosition(DOWN_DEGREES); }

    /** Move to the intermediate angle; repeated calls retain the same absolute position. */
    public void liftIntermediate() { setArmPosition(INTERMEDIATE_DEGREE); }

    /** Nudge the commanded angle, clamped between the calibrated UP and DOWN presets. */
    public void adjustDegrees(double deltaDegrees) {
        if (servo == null) throw new IllegalStateException("Call init() first");
        if (!Double.isFinite(deltaDegrees))
            throw new IllegalArgumentException("Angle change must be finite");
        double minimum = Math.min(UP_DEGREES, DOWN_DEGREES);
        double maximum = Math.max(UP_DEGREES, DOWN_DEGREES);
        setArmPosition(Math.max(minimum, Math.min(maximum, commandedArmDegrees + deltaDegrees)));
    }
    private void setArmPosition(double armDegrees) {
        if (servo == null) throw new IllegalStateException("Call IntakeArmSingleServo.init() first");
        double servoDegrees = FLOOR_OFFSET_DEGREES + armDegrees;
        if (servoDegrees < 0 || servoDegrees > SERVO_RANGE_DEGREES)
            throw new IllegalArgumentException("Arm calibration exceeds servo travel");
        servo.setPosition(servoDegrees / SERVO_RANGE_DEGREES);
        commandedArmDegrees = armDegrees;
    }

    /** Commanded angle, not a physical measurement of the arm. */
    public double getCommandedDegrees() { return commandedArmDegrees; }
}
