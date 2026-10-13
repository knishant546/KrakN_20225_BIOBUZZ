package org.firstinspires.ftc.teamcode.AutoAppl;

/** Shared draft settings. None of the poses or shooting values are field calibration. */
public final class HiveAutoConfig {
    private HiveAutoConfig() { }
    public static final boolean DRY_RUN = true;
    public static final boolean ROUTES_CALIBRATED = false;
    public static final boolean TAG_POSES_CALIBRATED = false;
    public static final boolean FEED_COUNTERS_CALIBRATED = false;
    public static final int PIPELINE = 1;
    public static final double VELOCITY = 1690, VELOCITY_TOLERANCE = 100;
    public static final double LIFT_DOWN = 0.55, LIFT_UP = 1.0;
    public static final double SPINUP = 1, FEED_TIME = 0.25;
    public static final double SETTLE = 0.8, ACQUIRE_TIMEOUT = 2.5, COLLECT_TIMEOUT = 3;
    // Starting estimate for emptying four balls, including spin-up and pose checks.
    // Tune on the robot: elapsed time does NOT confirm that four balls actually left.
    public static final double SHOOTING_TIMEOUT = 8;
    public static final double STOP_AT = 30, PARK_RESERVE = 7;
    public static final double PARK_AT = STOP_AT - PARK_RESERVE, MOVE_TIMEOUT = 6;
    public static final double POSITION_TOLERANCE = 2, HEADING_TOLERANCE = 5, PATH_SPEED = 0.25;
    public static final int POLLEN_CAPACITY = 4;
    // Same normalized sensor name and gain as KKN_Limelight / subsystems.ColorSensor.
    public static final String COLOR_SENSOR = "colorSensor";
    public static final float COLOR_GAIN = 20;
    // Initial thresholds only: calibrate with yellow pollen and the empty intake.
    public static final double COLOR_PRESENT_ALPHA = 0.07, COLOR_CLEAR_ALPHA = 0.04;
    public static final double YELLOW_MIN_HUE = 35, YELLOW_MAX_HUE = 85;
    public static final double YELLOW_MIN_SATURATION = 0.35;
    public static final double COLOR_DETECT_SECONDS = 0.04, COLOR_CLEAR_SECONDS = 0.08;
    // Limelight camera-space axis: 0=roll, 1=pitch, 2=yaw. NOT FTC cluster ftcPose.roll.
    public static final int POSE_AXIS = 0;
    // Calibrate each tag at the shooting pose; index = AprilTag ID minus 30.
    public static final double[] UPRIGHT_DEG = new double[16];
    public static final double[] AIM_TX_DEG = new double[16];
    public static final double UP_TOLERANCE = 30, DOWN_MIN = 120, AIM_TOLERANCE = 3;
    public static final double MIN_RANGE_M = 0.2, MAX_RANGE_M = 2.5;
    public static final int STABLE_FRAMES = 3;
    // Simulation ONLY. Set SIM_TIP_AFTER=1 to exercise stopping after the first feed attempt.
    public static final boolean SIM_INITIAL_UP = true;
    public static final int SIM_TIP_AFTER = 99;
}
