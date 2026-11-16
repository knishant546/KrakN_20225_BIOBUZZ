/* ============================================================================
 * What it does: Commands the positional ball gate OPEN/UP or CLOSED/DOWN.
 * Applications: BioBuzzTeleop and RED/BLUE Flower, Garden, Garden_NoFlwr Auto.
 * Replaces the continuous-rotation IntakeHelperServo; independent of intake.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public final class ShooterGateServo {
    public static final String HARDWARE_NAME = "ShooterGateServo";
    // PLACEHOLDERS: calibrate horn/linkage before loading the gate.
    // Assumes the existing goBILDA dual-mode servo is programmed for 300-degree positional travel.
    public static final double RANGE_DEGREES = 300.0;
    public static final double CLOSED_DEGREES = 0.0;
    public static final double OPEN_DEGREES = 40.0;
    public static final Servo.Direction DIRECTION = Servo.Direction.FORWARD;
    private Servo servo;
    private double commandedDegrees = Double.NaN;

    public void init(HardwareMap map) {
        servo = map.get(ServoImplEx.class, HARDWARE_NAME);
        servo.resetDeviceConfigurationForOpMode();
        servo.setDirection(DIRECTION);
        ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        commandedDegrees = Double.NaN;
        close();
    }
    public void open() { setDegrees(OPEN_DEGREES); }
    public void close() { setDegrees(CLOSED_DEGREES); }
    private void setDegrees(double degrees) {
        if (servo == null) return; // Allows cleanup after partial INIT failure.
        if (!Double.isFinite(degrees) || degrees < 0 || degrees > RANGE_DEGREES)
            throw new IllegalArgumentException("Gate calibration outside servo travel");
        if (Double.compare(commandedDegrees, degrees) != 0) {
            servo.setPosition(degrees / RANGE_DEGREES);
            commandedDegrees = degrees;
        }
    }
    public double getCommandedDegrees() { return commandedDegrees; }
    public boolean isOpen() { return servo != null && commandedDegrees == OPEN_DEGREES; }
}
