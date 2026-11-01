/* ============================================================================
 * Class: InputDCShooter
 * What it does:
 *   Starts/stops the power-controlled DC feeder using the legacy inputShooterServo hardware name.
 * Applications using it:
 *   BioBuzzTeleop
 *   BLUE_AutoFlower
 *   BLUE_AutoGarden
 *   RED_AutoFlower
 *   RED_AutoGarden
 *   Includes indirect/source dependencies; optional features may be inactive.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Start/stop feeder using a DC motor.
 * Legacy hardware name retained; configure this device on a DC motor port.
 * Direct methods work with ordinary FTC OpModes without a NextFTC scheduler.
 */
public class InputDCShooter {
    public static final String HARDWARE_NAME = "inputShooterServo";
    private DcMotor motor;
    private double runningPower = -0.50;
    private boolean running;

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, HARDWARE_NAME);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        stop();
    }

    /** Sets the signed speed for the next start, matching Spinner's stored-power approach. */
    public void setPower(double power) {
        if (!Double.isFinite(power) || power < -1 || power > 1)
            throw new IllegalArgumentException("Motor power must be between -1 and 1");
        runningPower = power;
    }

    public void start() {
        if (motor == null) throw new IllegalStateException("Call init() first");
        motor.setPower(runningPower);
        running = runningPower != 0;
    }

    public void stop() {
        if (motor != null) motor.setPower(0);
        running = false;
    }

    public boolean isRunning() { return running; }
}
