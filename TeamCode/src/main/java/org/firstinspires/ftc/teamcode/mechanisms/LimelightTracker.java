package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Simple Limelight helper for BIOBUZZ ball following.
 *
 * Limelight pipeline setup expected:
 *   Pipeline 0 = Yellow POLLEN color pipeline
 *   Pipeline 1 = Red NECTAR color pipeline
 *
 * The class:
 *   1. Reads the active Limelight color pipeline.
 *   2. Chooses the largest detected color blob.
 *   3. Uses tx to strafe left/right toward the ball.
 *   4. Drives forward slowly while the ball is not yet close.
 *
 * NOTE: YELLOW_STOP_AREA and RED_STOP_AREA are STARTING VALUES ONLY.
 * Calibrate them on your robot using telemetry.
 */
public class LimelightTracker {

    public enum TargetType {
        YELLOW_POLLEN,
        RED_NECTAR
    }

    private static final int YELLOW_PIPELINE = 0;
    private static final int RED_PIPELINE = 1;

    // Motion tuning - intentionally slow for first testing.
    private static final double STRAFE_KP = 0.025;
    private static final double MAX_STRAFE = 0.30;
    private static final double MAX_FORWARD = 0.20;
    private static final double ALIGNMENT_DEADBAND_DEG = 2.0;
    private static final double FORWARD_ALIGNMENT_LIMIT_DEG = 12.0;
    private static final double MIN_VALID_AREA = 0.15;

    /*
     * Starting guesses only.
     * Red Nectar is physically larger than Yellow Pollen, so at the same
     * distance its image area should normally be larger.
     *
     * Put each ball at the desired pickup point and replace these values
     * with the telemetry Target Area measured on your robot.
     */
    private double yellowStopArea = 8.0;
    private double redStopArea = 13.0;

    /*
     * If the robot strafes AWAY from the ball instead of toward it,
     * change this from +1.0 to -1.0.
     */
    private static final double STRAFE_DIRECTION = 1.0;

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private TargetType targetType = TargetType.YELLOW_POLLEN;
    private int activePipeline = -1;

    private boolean targetVisible = false;
    private double tx = 0.0;
    private double ty = 0.0;
    private double area = 0.0;

    public LimelightTracker(HardwareMap hardwareMap, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = telemetry;

        // 50 Hz is plenty for a first drivetrain-follow test.
        limelight.setPollRateHz(50);

        setTargetType(TargetType.YELLOW_POLLEN);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public boolean isRunning() {
        return limelight.isRunning();
    }

    public void setTargetType(TargetType newTargetType) {
        if (newTargetType == null) {
            return;
        }

        targetType = newTargetType;

        int wantedPipeline =
                (targetType == TargetType.YELLOW_POLLEN)
                        ? YELLOW_PIPELINE
                        : RED_PIPELINE;

        // Avoid requesting the same pipeline switch every OpMode loop.
        if (wantedPipeline != activePipeline) {
            limelight.pipelineSwitch(wantedPipeline);
            activePipeline = wantedPipeline;
        }
    }

    public TargetType getTargetType() {
        return targetType;
    }

    /**
     * Read the newest Limelight result and select the largest valid color blob.
     *
     * @return true if a target is currently visible.
     */
    public boolean update() {
        targetVisible = false;
        tx = 0.0;
        ty = 0.0;
        area = 0.0;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            addTelemetry();
            return false;
        }

        List<LLResultTypes.ColorResult> targets = result.getColorResults();

        if (targets == null || targets.isEmpty()) {
            addTelemetry();
            return false;
        }

        LLResultTypes.ColorResult bestTarget = null;
        double largestArea = 0.0;

        for (LLResultTypes.ColorResult candidate : targets) {
            double candidateArea = candidate.getTargetArea();

            if (candidateArea >= MIN_VALID_AREA &&
                    candidateArea > largestArea) {
                largestArea = candidateArea;
                bestTarget = candidate;
            }
        }

        if (bestTarget == null) {
            addTelemetry();
            return false;
        }

        targetVisible = true;
        tx = bestTarget.getTargetXDegrees();
        ty = bestTarget.getTargetYDegrees();
        area = bestTarget.getTargetArea();

        addTelemetry();
        return true;
    }

    /**
     * Follow the selected ball using Mecanum strafe + slow forward motion.
     *
     * Hold a gamepad button while calling this method.
     * Releasing the button should return control to manual drive.
     */
    public void followTarget(MecanumDrive drive) {
        if (!update()) {
            drive.drive(0.0, 0.0, 0.0);
            return;
        }

        double stopArea =
                (targetType == TargetType.YELLOW_POLLEN)
                        ? yellowStopArea
                        : redStopArea;

        // Stop when the ball is close enough to the intake.
        if (area >= stopArea) {
            drive.drive(0.0, 0.0, 0.0);
            telemetry.addData("Limelight Follow", "Target close - STOP");
            return;
        }

        // tx controls left/right Mecanum movement.
        double strafe = 0.0;

        if (Math.abs(tx) > ALIGNMENT_DEADBAND_DEG) {
            strafe = Range.clip(
                    STRAFE_DIRECTION * tx * STRAFE_KP,
                    -MAX_STRAFE,
                    MAX_STRAFE);
        }

        /*
         * Drive forward faster when reasonably aligned.
         * If the ball is far to one side, slow forward movement so the
         * robot first has time to get the intake behind the ball.
         */
        double forward;

        if (Math.abs(tx) <= FORWARD_ALIGNMENT_LIMIT_DEG) {
            forward = MAX_FORWARD;
        } else {
            forward = MAX_FORWARD * 0.40;
        }

        // Keep heading fixed for this simple test: rotate = 0.
        drive.drive(forward, strafe, 0.0);

        telemetry.addData("Follow Forward", "%.2f", forward);
        telemetry.addData("Follow Strafe", "%.2f", strafe);
    }

    public boolean hasTarget() {
        return targetVisible;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getArea() {
        return area;
    }

    public void setStopAreas(double yellowStopArea, double redStopArea) {
        this.yellowStopArea = yellowStopArea;
        this.redStopArea = redStopArea;
    }

    private void addTelemetry() {
        telemetry.addData("LL Connected", limelight.isConnected());
        telemetry.addData("LL Running", limelight.isRunning());
        telemetry.addData("LL Target Type", targetType);
        telemetry.addData("LL Target Visible", targetVisible);

        if (targetVisible) {
            telemetry.addData("LL tx", "%.2f deg", tx);
            telemetry.addData("LL ty", "%.2f deg", ty);
            telemetry.addData("LL Target Area", "%.2f", area);
        }
    }
}
