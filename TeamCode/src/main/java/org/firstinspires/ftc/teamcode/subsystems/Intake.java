/* ============================================================================
 * Class: Intake
 * What it does:
 *   Controls the intake DC motor and synchronized continuous-rotation helper servo.
 * Applications using it:
 *   AutoBlueD1 [disabled draft]
 *   AutoBlueD6 [disabled draft]
 *   AutoRedBack [excluded from build]
 *   AutoRedC1 [disabled draft]
 *   AutoRedC6 [disabled draft]
 *   BioBuzzTeleop
 *   BLUE_AutoFlower
 *   BLUE_AutoGarden
 *   DecodeTeleop [excluded from build]
 *   KKN_TeleOP
 *   RED_AutoFlower
 *   RED_AutoGarden
 *   Includes indirect/source dependencies; optional features may be inactive.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake  {

    private DcMotor intakeMotor;
    private boolean helperEnabled, reverseHelper;
    private final IntakeHelperServo helperServo = new IntakeHelperServo();

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, true); // TeleOp keeps the synchronized helper.
    }

    /** Optional helper hardware; default direction matches TeleOp. */
    public void init(HardwareMap hardwareMap, boolean enableHelper) {
        init(hardwareMap, enableHelper, false);
    }

    /** Reverse only the helper for autonomous; DC intake direction is unchanged. */
    public void init(HardwareMap hardwareMap, boolean enableHelper, boolean reverseHelper) {
        this.reverseHelper = reverseHelper;
        helperServo.stop();
        helperEnabled = enableHelper;
        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setPower(0);
        if (helperEnabled) helperServo.init(hardwareMap); // INIT leaves actuators stopped.
    }

    public void  start() {
        start(1.0);

    }

    /** Run at a fraction of full power, retaining the intake direction. */
    public void start(double powerFraction) {
        start(powerFraction, false);
    }

    /** Reverse changes the power sign; existing callers keep normal intake direction. */
    public void start(double powerFraction, boolean reverse) {
        start(powerFraction, reverse, true);
    }

    /** Allow shooting to run only the DC intake, while collection retains the helper. */
    public void start(double powerFraction, boolean reverse, boolean runHelper) {
        if (!Double.isFinite(powerFraction) || powerFraction < 0 || powerFraction > 1)
            throw new IllegalArgumentException("Intake power fraction must be 0..1");
        intakeMotor.setPower(reverse ? powerFraction : -powerFraction);
        // Helper selection is independent of DC speed adjustments; zero DC power stops both.
        if (helperEnabled && runHelper && powerFraction > 0) helperServo.start(reverse ^ reverseHelper);
        else helperServo.stop();
    }

    /** Select helper direction for the next start call, without changing the DC motor. */
    public void setHelperReverse(boolean reverse) { reverseHelper = reverse; }

    /** Select helper power without changing the DC motor or starting either actuator. */
    public void setHelperPower(double fraction) { helperServo.setPowerFraction(fraction); }
    public void stop() {
        helperServo.stop();
        if (intakeMotor != null) intakeMotor.setPower(0.0);

    }
}
