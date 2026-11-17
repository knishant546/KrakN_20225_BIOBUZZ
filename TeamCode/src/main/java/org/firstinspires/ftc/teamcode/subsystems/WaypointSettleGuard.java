package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;

/** Near-target fallback for RED/BLUE Garden, Garden_NoFlwr and Flower moves.
 * Does not replace normal follower completion, timed backward tests or hard deadlines.
 */
public final class WaypointSettleGuard {
    private double enteredAt = Double.NaN;
    public void reset() { enteredAt = Double.NaN; }
    public boolean update(double now, double distanceInches, double headingErrorRadians) {
        double angle = Math.abs(Math.atan2(Math.sin(headingErrorRadians), Math.cos(headingErrorRadians)));
        if (!Double.isFinite(distanceInches) || !Double.isFinite(angle)
                || distanceInches > WAYPOINT_NEAR_POSITION_INCHES
                || angle > Math.toRadians(WAYPOINT_NEAR_HEADING_DEGREES)) {
            reset(); return false;
        }
        if (Double.isNaN(enteredAt)) enteredAt = now;
        return now-enteredAt >= WAYPOINT_NEAR_HOLD_SECONDS;
    }
}