/* ============================================================================
 * Class: IntakeHelperServo
 * What it does: Runs the continuous-rotation intake helper at adjustable power (80% default).
 * Applications using it: Intake; indirectly BioBuzzTeleop, ShooterShotCalibration
 *   and autonomous programs using the shared Intake subsystem.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Servo must be physically programmed for continuous rotation and configured as CRServo. */
public class IntakeHelperServo {
    public static final String HARDWARE_NAME = "IntakeHelperServo";
    public static final double RUN_POWER = 0.80;
    // Change FORWARD to REVERSE if the mounted helper feeds outward in normal operation.
    public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
    private CRServo servo;
    private double selectedPower = RUN_POWER;

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, HARDWARE_NAME);
        selectedPower = RUN_POWER;
        servo.setPower(0);
        servo.setDirection(DIRECTION);
    }
    public void start(boolean reverse) {
        servo.setPower(reverse ? -selectedPower : selectedPower);
    }
    /** Changes the next start command only; never starts a stopped servo. */
    public void setPowerFraction(double power) {
        if (!Double.isFinite(power) || power < 0 || power > 1)
            throw new IllegalArgumentException("Helper power must be 0..1");
        selectedPower = power;
    }
    public void stop() {
        if (servo != null) servo.setPower(0);
    }
}