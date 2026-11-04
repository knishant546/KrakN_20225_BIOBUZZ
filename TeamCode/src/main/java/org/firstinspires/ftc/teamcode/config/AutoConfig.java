/* ============================================================================
 * Class: AutoConfig
 * What it does: Central tuning values for autonomous mechanisms, paths, and timing.
 * Applications using it: RED/BLUE_AutoFlower and RED/BLUE_AutoGarden,
 *   through AutoShootSequence and Pedro Constants where appropriate.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.math.Pose;

/** Edit here, rebuild, and install. Fractions are 0..1; durations are seconds. */
public final class AutoConfig {
    private AutoConfig() { }
    // Shooter fraction scales ShooterBBTeleop's encoder velocity reference, not motor power.
    public static final double SHOOTER_SPEED_FRACTION = 0.60;
    public static final double INTAKE_POWER = 1.00;
    public static final double FEEDER_POWER = -0.50; // Signed DC motor power.
    public static final double MAX_PATH_SPEED = 1.00;
    public static final double FEEDER_DELAY_SECONDS = 1.0;
    public static final double DEFAULT_SHOOT_SECONDS = 6.0;
    public static final double PARK_DEADLINE_SECONDS = 24.0;
    // Driving only: collection, shooting, and vision waits use separate timers.
    public static final double WAYPOINT_TIMEOUT_SECONDS = 3.0;
    public static final double AUTO_LIMIT_SECONDS = 30.0;

    public static final double GARDEN_FIRST_SHOOT_SECONDS = 4.0; // Includes feeder startup.
    public static final double GARDEN_SECOND_SHOOT_SECONDS = 3.0; // Includes feeder spin-up.
    public static final double GARDEN_COLLECTION_PAUSE_SECONDS = 0.5;
    public static final double GARDEN_COLLECTION_SECONDS = 2.0;
    public static final double GARDEN_HIVE_WAIT_SECONDS = 2.0;
    public static final double GARDEN_HIVE_STABLE_SECONDS = 1.0;
    public static final double GARDEN_FLOWER_COLLECTION_SECONDS = 2.0;
    public static final double GARDEN_THIRD_SHOOT_SECONDS = 3.0;
    public static final double FLOWER_FIRST_SHOOT_SECONDS = 5.0;
    public static final double FLOWER_SECOND_SHOOT_SECONDS = 6.0;
    public static final double HIVE_WAIT_TIMEOUT_SECONDS = 10.0; // Flower first shot.
    public static final double HIVE_SETTLE_SECONDS = 3.0; // Flower continuous tag visibility.
    public static final double FLOWER_ARM_PAUSE_SECONDS = 1.0;
    public static final double FLOWER_SECOND_POSITION_PAUSE_SECONDS = 2.0;
    public static final double BACKWARD_TEST_SECONDS = 1.0; // Flower timed backward leg.

    // Start-relative coordinates: inches, heading DEGREES. BLUE starts physically
    // rotated 180 degrees from RED, then resets local pose to (0,0,0).
    // Flower and Garden coordinates and timings are shared by both alliances.
    public static final Waypoint START = new Waypoint(0, 0, 0);
    public static final Waypoint FLOWER_SHOOT_1 = new Waypoint(-1, 0, 0);
    public static final Waypoint FLOWER_STRAIGHT_END = new Waypoint(-10, 0, 0);
    public static final Waypoint FLOWER_STRAFE_END = new Waypoint(-10, 10, 0);
    public static final Waypoint FLOWER_SHOOT_2 = new Waypoint(-5, 10, 0);
    public static final Waypoint FLOWER_EXIT_STRAIGHT = new Waypoint(-11, 10, 0);
    public static final Waypoint FLOWER_PARK = new Waypoint(-11, 42, 0);
    public static final Waypoint GARDEN_SHOOT = new Waypoint(-1, 0, 0);

    // Both alliances use the same start-relative Garden route.

    // Garden collection maneuver starts here (RED and BLUE).
    public static final Waypoint GARDEN_APPROACH_START = new Waypoint(-4, 0, 0);
    //Garden Collection point reached
    public static final Waypoint GARDEN_COLLECT_REACH = new Waypoint(-4, -52.90, 0);
    //Slide Inside Garden to pick the balls
    public static final Waypoint GARDEN_COLLECT_SLIDE_IN = new Waypoint(-0.7, -52.90, 0);

    //start Intake only motor for 2 secs

    //Prepare to return- Move forward 1 inches
    public static final Waypoint GARDEN_COLLECT_SLIDE_OUT = new Waypoint(-1, -54.0, 0);

    //RETURN to Shooting position
    public static final Waypoint GARDEN_SHOOT_HOME = new Waypoint(-1, 0, 0);

    //Shoot for 3 secs

    // Seq
    //public static final Waypoint GARDEN_CLEARANCE = new Waypoint(-1, -3, 0);
    // RED Garden only: strafe 3 more inches in -Y before the diagonal parking leg.
   // public static final Waypoint RED_GARDEN_EXTRA_CLEARANCE = new Waypoint(-1, -6, 0);
    //public static final Waypoint GARDEN_PARK = new Waypoint(-73.16, -45.13, -91.24);

    // Both alliances: optional flower trip after completing the confirmed second shot.
    public static final Waypoint GARDEN_FLOWER_START = new Waypoint(-4, 0, 0);
    public static final Waypoint GARDEN_FLOWER_REACHED = new Waypoint(-26.36, -36.74, 270);
    public static final Waypoint GARDEN_FLOWER_SLIDE_IN = new Waypoint(-25.97, -43.89, 270);

    // Before park strafe, so that you do not hit Hive poles
    public static final Waypoint GARDEN_BEFOREPARK_STRAFE = new Waypoint(-4, -23, 0);
    public static final Waypoint GARDEN_PARK = new Waypoint(-88.00, -45, 0);

    /** Immutable tuning entry; used by the four Auto programs when building paths. */
    public static final class Waypoint {
        public final double x, y, headingDegrees;
        private Waypoint(double x, double y, double headingDegrees) {
            this.x = x; this.y = y; this.headingDegrees = headingDegrees;
        }
        public Pose pose() { return new Pose(x, y, Math.toRadians(headingDegrees)); }
    }
}