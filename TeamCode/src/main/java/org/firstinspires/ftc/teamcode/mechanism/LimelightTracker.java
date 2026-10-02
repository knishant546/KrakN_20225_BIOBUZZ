package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * Simple Limelight helper for BIOBUZZ ball following.
 *
 * Limelight pipeline setup expected:
 *   Pipeline 1 = AprilTag pipeline with Full 3D Targeting enabled
 *   Pipeline 2 = Yellow POLLEN neural detector (class label "pollen")
 *   Pipeline 8 = Red NECTAR color pipeline
 *
 * The class:
 *   1. Reads the active Limelight detector or color pipeline.
 *   2. Chooses the largest qualifying pollen detection or red color blob.
 *   3. Uses tx to strafe left/right toward the ball.
 *   4. Drives forward slowly while the ball is not yet close.
 *
 * NOTE: YELLOW_STOP_AREA and RED_STOP_AREA are STARTING VALUES ONLY.
 * Calibrate them on your robot using telemetry.
 */
public class LimelightTracker {

    public enum TargetType {
        YELLOW_POLLEN,
        RED_NECTAR,
        APRILTAG
    }

    private static final int YELLOW_PIPELINE = 2;
    private static final int RED_PIPELINE = 8;
    // Set this to the AprilTag pipeline configured in the Limelight web UI.
    public static final int APRILTAG_PIPELINE = 1;
    private static final long MAX_RESULT_AGE_MS = 250;

    // Motion tuning - intentionally slow for first testing.
    private static final double STRAFE_KP = 0.025;
    private static final double MAX_STRAFE = 0.30;
    private static final double MAX_FORWARD = 0.20;
    private static final double ALIGNMENT_DEADBAND_DEG = 2.0;
    private static final double FORWARD_ALIGNMENT_LIMIT_DEG = 12.0;
    private static final double MIN_VALID_AREA = 0.15;
    private static final double MIN_POLLEN_CONFIDENCE = 0.51;
    private static final String POLLEN_CLASS_NAME = "pollen";
    // This camera's LLOS 2024.10.1 detector reports fractional area:
    // measured detector ta ~0.006413 vs overall ta 0.6413 percent.
    // SDK passes that value through despite documenting percent units.
    // Recheck against overall ta after firmware changes; do not auto-scale by size.
    private static final double DETECTOR_AREA_TO_PERCENT = 100.0;

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
    private final double[] areaSamples = new double[5];
    private int areaSampleCount = 0;
    private int areaSampleIndex = 0;
    private double lastAreaTimestamp = Double.NaN;
    private double smoothedArea = 0.0;
    private boolean stoppedAtTarget = false;
    // Resume only below 85% of the calibrated stop area to avoid chatter.
    private static final double RESUME_AREA_RATIO = 0.85;

    private void resetAreaFilter() {
        areaSampleCount = 0;
        areaSampleIndex = 0;
        lastAreaTimestamp = Double.NaN;
        smoothedArea = 0.0;
        stoppedAtTarget = false;
    }

    private void updateAreaFilter(double timestamp) {
        // An OpMode loop may read the same camera frame more than once.
        if (Double.compare(timestamp, lastAreaTimestamp) == 0) {
            return;
        }
        lastAreaTimestamp = timestamp;
        areaSamples[areaSampleIndex] = area;
        areaSampleIndex = (areaSampleIndex + 1) % areaSamples.length;
        areaSampleCount = Math.min(areaSampleCount + 1, areaSamples.length);
        double sum = 0.0;
        for (int i = 0; i < areaSampleCount; i++) {
            sum += areaSamples[i];
        }
        smoothedArea = sum / areaSampleCount;
    }

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

        int wantedPipeline = targetType == TargetType.APRILTAG
                ? APRILTAG_PIPELINE
                : (targetType == TargetType.YELLOW_POLLEN ? YELLOW_PIPELINE : RED_PIPELINE);

        // Avoid requesting the same pipeline switch every OpMode loop.
        if (wantedPipeline != activePipeline) {
            resetAreaFilter();
            limelight.pipelineSwitch(wantedPipeline);
            activePipeline = wantedPipeline;
        }
    }

    public TargetType getTargetType() {
        return targetType;
    }

    /**
     * Read the newest result and select a target for the active pipeline.
     *
     * @return true if a target is currently visible.
     */
    public boolean update() {
        targetVisible = false;
        tx = 0.0;
        ty = 0.0;
        area = 0.0;

        LLResult result = getFreshResult();

        if (result == null) {
            addTelemetry();
            return false;
        }

        if (targetType == TargetType.APRILTAG) {
            targetVisible = addAprilTagTelemetry(result);
            addTelemetry();
            return targetVisible;
        }

        if (targetType == TargetType.YELLOW_POLLEN) {
            targetVisible = updatePollenDetector(result);
            addTelemetry();
            return targetVisible;
        }

        List<LLResultTypes.ColorResult> targets = result.getColorResults();
        telemetry.addData("LL Result ta (%)", "%.4f", result.getTa());
        telemetry.addData("LL Min Candidate Area (%)", "%.4f", MIN_VALID_AREA);
        telemetry.addData("LL Color Candidate Count", targets == null ? 0 : targets.size());

        if (targets == null || targets.isEmpty()) {
            telemetry.addData("LL Color Selection", "No color candidates returned");
            addTelemetry();
            return false;
        }

        LLResultTypes.ColorResult bestTarget = null;
        double largestArea = 0.0;
        int candidateIndex = 0;
        int selectedIndex = -1;

        for (LLResultTypes.ColorResult candidate : targets) {
            double candidateArea = candidate.getTargetArea();
            telemetry.addData("LL Candidate " + candidateIndex + " Area (%)",
                    "%.4f | %s", candidateArea,
                    candidateArea >= MIN_VALID_AREA ? "PASS cutoff" : "REJECT cutoff");

            if (candidateArea >= MIN_VALID_AREA &&
                    candidateArea > largestArea) {
                largestArea = candidateArea;
                bestTarget = candidate;
                selectedIndex = candidateIndex;
            }
            candidateIndex++;
        }

        if (bestTarget == null) {
            telemetry.addData("LL Color Selection", "No candidate passes area cutoff");
            addTelemetry();
            return false;
        }

        targetVisible = true;
        telemetry.addData("LL Color Selection", "Candidate %d (largest passing area)", selectedIndex);
        tx = bestTarget.getTargetXDegrees();
        ty = bestTarget.getTargetYDegrees();
        area = bestTarget.getTargetArea();
        updateAreaFilter(result.getTimestamp());

        addTelemetry();
        return true;
    }

    private boolean updatePollenDetector(LLResult result) {
        List<LLResultTypes.DetectorResult> targets = result.getDetectorResults();
        telemetry.addData("LL Result ta (%)", "%.4f", result.getTa());
        telemetry.addData("LL Detector Count", targets == null ? 0 : targets.size());
        telemetry.addData("LL Detector Cutoffs", "confidence >= %.2f, area >= %.4f%%",
                MIN_POLLEN_CONFIDENCE, MIN_VALID_AREA);
        if (targets == null || targets.isEmpty()) {
            telemetry.addData("LL Detector Selection", "No detector results - check pipeline 2 model");
            return false;
        }

        LLResultTypes.DetectorResult best = null;
        int bestIndex = -1;
        for (int i = 0; i < targets.size(); i++) {
            LLResultTypes.DetectorResult candidate = targets.get(i);
            String label = candidate.getClassName();
            double confidence = candidate.getConfidence();
            double rawDetectorArea = candidate.getTargetArea();
            double candidateArea = rawDetectorArea * DETECTOR_AREA_TO_PERCENT;
            String reason;
            if (label == null || !POLLEN_CLASS_NAME.equalsIgnoreCase(label.trim())) {
                reason = "REJECT class";
            } else if (!Double.isFinite(confidence) || confidence < MIN_POLLEN_CONFIDENCE) {
                reason = "REJECT confidence";
            } else if (!Double.isFinite(candidateArea) || candidateArea < MIN_VALID_AREA) {
                reason = "REJECT area";
            } else if (!Double.isFinite(candidate.getTargetXDegrees())
                    || !Double.isFinite(candidate.getTargetYDegrees())) {
                reason = "REJECT angles";
            } else {
                reason = "PASS";
                if (best == null || rawDetectorArea > best.getTargetArea()) {
                    best = candidate;
                    bestIndex = i;
                }
            }
            telemetry.addData("LL Detector " + i,
                    "%s | conf %.3f | raw %.6f | %.4f%% | %s",
                    label, confidence, rawDetectorArea, candidateArea, reason);
        }
        if (best == null) {
            telemetry.addData("LL Detector Selection", "No pollen passes cutoffs");
            return false;
        }
        tx = best.getTargetXDegrees();
        ty = best.getTargetYDegrees();
        area = best.getTargetArea() * DETECTOR_AREA_TO_PERCENT;
        updateAreaFilter(result.getTimestamp());
        telemetry.addData("LL Detector Selection", "Candidate %d: %s", bestIndex, best.getClassName());
        telemetry.addData("LL Pollen Confidence", "%.3f", best.getConfidence());
        return true;
    }

    private LLResult getFreshResult() {
        LLResult result = limelight.getLatestResult();
        // Diagnostic values only: a rejected frame must never command motion.
        // Show these before validation so missing area telemetry has an explanation.
        if (result == null) {
            telemetry.addData("LL Raw Frame ta (%)", "N/A - no camera result");
        } else {
            telemetry.addData("LL Raw Frame ta (%)", "%.4f", result.getTa());
            telemetry.addData("LL Raw Frame Valid", result.isValid());
            telemetry.addData("LL Raw Pipeline Type", result.getPipelineType());
        }
        if (!limelight.isConnected() || !limelight.isRunning() || result == null) {
            telemetry.addData("LL Frame", "Camera unavailable / waiting for data");
            return null;
        }
        telemetry.addData("LL Returned Pipeline", result.getPipelineIndex());
        telemetry.addData("LL Frame Age (ms)", result.getStaleness());
        if (result.getPipelineIndex() != activePipeline) {
            telemetry.addData("LL Frame", "Waiting for requested pipeline");
            return null;
        }
        if (result.getStaleness() > MAX_RESULT_AGE_MS) {
            telemetry.addData("LL Frame", "Stale - ignoring result");
            return null;
        }
        if (!result.isValid()) {
            telemetry.addData("LL Frame", "No target detected");
            return null;
        }
        telemetry.addData("LL Frame", "Valid");
        return result;
    }

    /**
     * Report each tag's pose relative to the camera, not field-space robot pose.
     * The SDK reports rotations about camera X/Y/Z as roll/pitch/yaw.
     * Display all three because a change in viewing angle need not change roll.
     */
    private boolean addAprilTagTelemetry(LLResult result) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        telemetry.addData("AprilTag Count", tags == null ? 0 : tags.size());
        if (tags == null || tags.isEmpty()) {
            telemetry.addData("AprilTag", "No tags detected");
            return false;
        }
        telemetry.addData("AprilTag Pose Frame", "Target relative to CAMERA");
        for (LLResultTypes.FiducialResult tag : tags) {
            String label = "Tag " + tag.getFiducialId();
            telemetry.addData(label + " tx/ty (deg)", "%.2f / %.2f",
                    tag.getTargetXDegrees(), tag.getTargetYDegrees());
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) {
                telemetry.addData(label + " Pose", "Unavailable - enable Full 3D Targeting");
                continue;
            }
            Position position = pose.getPosition().toUnit(DistanceUnit.METER);
            YawPitchRollAngles orientation = pose.getOrientation();
            // The SDK substitutes a zero pose when the camera omits 3D data.
            if (!Double.isFinite(position.x) || !Double.isFinite(position.y)
                    || !Double.isFinite(position.z)
                    || (position.x == 0 && position.y == 0 && position.z == 0)) {
                telemetry.addData(label + " Pose", "Unavailable - check Full 3D Targeting");
                continue;
            }
            telemetry.addData(label + " XYZ (m)", "%.3f / %.3f / %.3f",
                    position.x, position.y, position.z);
            telemetry.addData(label + " Roll (deg)", "%.2f", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData(label + " Pitch (deg)", "%.2f", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData(label + " Yaw (deg)", "%.2f", orientation.getYaw(AngleUnit.DEGREES));
        }
        return true;
    }

    /**
     * Follow the selected ball using Mecanum strafe + slow forward motion.
     *
     * Hold a gamepad button while calling this method.
     * Releasing the button should return control to manual drive.
     */
    public void followTarget(MecanumDrive drive) {
        // AprilTag mode is telemetry-only; never use a tag as a color-follow target.
        if (targetType == TargetType.APRILTAG) {
            drive.drive(0.0, 0.0, 0.0);
            update();
            return;
        }
        if (!update()) {
            drive.drive(0.0, 0.0, 0.0);
            return;
        }

        double stopArea =
                (targetType == TargetType.YELLOW_POLLEN)
                        ? yellowStopArea
                        : redStopArea;

        // Stop when the ball is close enough to the intake.
        // Stop immediately on raw area; filtering must not delay stopping.
        if (area >= stopArea) {
            stoppedAtTarget = true;
        } else if (area < stopArea * RESUME_AREA_RATIO
                && smoothedArea < stopArea * RESUME_AREA_RATIO) {
            stoppedAtTarget = false;
        }
        if (stoppedAtTarget) {
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
        // Never carry old area samples through target loss or AprilTag mode.
        if (!targetVisible || targetType == TargetType.APRILTAG) {
            resetAreaFilter();
        }
        telemetry.addData("LL Connected", limelight.isConnected());
        telemetry.addData("LL Running", limelight.isRunning());
        telemetry.addData("LL Target Type", targetType);
        telemetry.addData("LL Requested Pipeline", activePipeline);
        telemetry.addData("LL Target Visible", targetVisible);

        if (targetVisible && targetType != TargetType.APRILTAG) {
            telemetry.addData("LL tx", "%.2f deg", tx);
            telemetry.addData("LL ty", "%.2f deg", ty);
            telemetry.addData("LL Target Area", "%.2f", area);
            telemetry.addData("LL Area Average (5 frames)", "%.3f", smoothedArea);
            double stopArea = targetType == TargetType.YELLOW_POLLEN ? yellowStopArea : redStopArea;
            telemetry.addData("LL Stop / Resume Area", "%.3f / %.3f",
                    stopArea, stopArea * RESUME_AREA_RATIO);
        }
    }
}
