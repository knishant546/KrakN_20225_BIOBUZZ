package org.firstinspires.ftc.teamcode.subsystems;

/** Time-based wheel-command ramp used by BioBuzzTeleop through MecanumDrive.
 * One shared interpolation fraction preserves the requested wheel-change ratios.
 * Explicit zero bypasses ramping; elapsed time is capped to avoid jumps after a stalled loop.
 */
public final class DriveSlewLimiter {
    private final double[] output = new double[4];
    private double lastTime = Double.NaN;
    public double[] update(double[] target, double now, double cap, double seconds, boolean enabled) {
        double dt = Double.isNaN(lastTime) ? 0 : Math.max(0, Math.min(0.05, now-lastTime));
        lastTime = now;
        boolean zero = true;
        double maxChange = 0;
        for (int i=0; i<4; i++) {
            zero &= target[i] == 0;
            maxChange = Math.max(maxChange, Math.abs(target[i]-output[i]));
        }
        double fraction = !enabled || zero || seconds <= 0 || maxChange == 0
                ? 1 : Math.min(1, cap / seconds * dt / maxChange);
        for (int i=0; i<4; i++) output[i] += (target[i]-output[i])*fraction;
        return output.clone();
    }
    public void reset() {
        java.util.Arrays.fill(output, 0);
        lastTime = Double.NaN;
    }
}