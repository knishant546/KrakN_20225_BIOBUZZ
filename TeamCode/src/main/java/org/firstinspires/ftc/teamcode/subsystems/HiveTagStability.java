package org.firstinspires.ftc.teamcode.subsystems;
/* ============================================================================
 * What it does: Accumulates good-frame time, pausing during brief tag loss.
 * Used by: HiveUpTagIDLogic for RED/BLUE Flower and Garden autonomous gates.
 * ============================================================================ */
public final class HiveTagStability {
    private long previous = -1, lastGood = -1, accumulated;
    private boolean visible;
    public void reset() { previous = lastGood = -1; accumulated = 0; visible = false; }
    public void observe(long now, boolean qualifies, long allowanceNanos) {
        // Bound gaps from the last good frame; repeated bad frames cannot extend grace.
        if (lastGood >= 0 && now - lastGood > allowanceNanos) reset();
        if (qualifies) {
            if (visible && previous >= 0) accumulated += now - previous;
            lastGood = now;
        }
        // Intervals touching a missing-tag frame do not earn stability time.
        visible = qualifies;
        previous = now;
    }
    public double seconds() { return accumulated / 1e9; }
    public boolean ready(double seconds) { return visible && seconds() >= seconds; }
}
