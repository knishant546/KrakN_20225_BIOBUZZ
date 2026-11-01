/* ============================================================================
 * Class: Intake
 * What it does:
 *   Controls intake DC motor power and reverses rotation by changing the power sign.
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

    public void init(HardwareMap hardwareMap) {
        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake");
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
        if (!Double.isFinite(powerFraction) || powerFraction < 0 || powerFraction > 1)
            throw new IllegalArgumentException("Intake power fraction must be 0..1");
        intakeMotor.setPower(reverse ? powerFraction : -powerFraction);
    }

    public void stop() {
        intakeMotor.setPower(0.0);

    }
}
