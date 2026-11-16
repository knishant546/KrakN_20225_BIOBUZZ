/* ============================================================================
 * What it does: Controls the intake DC motor. The former helper servo is now
 *   ShooterGateServo and is controlled independently by shooting logic.
 * Applications: BioBuzzTeleop, AutoShootSequence and legacy test applications.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    public void init(HardwareMap map) {
        intakeMotor = map.get(DcMotor.class,"intake");
        intakeMotor.setPower(0);
    }
    /** Legacy source compatibility: former helper options no longer map any servo. */
    public void init(HardwareMap map, boolean unusedHelper) { init(map); }
    public void init(HardwareMap map, boolean unusedHelper, boolean unusedReverse) { init(map); }
    public void start() { start(1.0); }
    public void start(double power) { start(power,false); }
    public void start(double power, boolean reverse) {
        if (!Double.isFinite(power) || power < 0 || power > 1)
            throw new IllegalArgumentException("Intake power must be 0..1");
        intakeMotor.setPower(reverse?power:-power);
    }
    public void start(double power, boolean reverse, boolean unusedHelper) { start(power,reverse); }
    public void stop() { if (intakeMotor != null) intakeMotor.setPower(0); }
}
