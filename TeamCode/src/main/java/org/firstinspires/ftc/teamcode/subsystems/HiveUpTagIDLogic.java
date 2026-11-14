/* ============================================================================
 * Class: HiveUpTagIDLogic
 * What it does:
 *   Counts fresh alliance-specific AprilTags on pipeline 0 as a hive-UP visibility heuristic.
 * Applications using it:
 *   BLUE_AutoFlower
 *   BLUE_AutoGarden
 *   RED_AutoFlower
 *   RED_AutoGarden
 *   Includes indirect/source dependencies; optional features may be inactive.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.HIVE_TAG_DROPOUT_MS;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Locale;
import java.util.HashSet;
import java.util.Set;

/** Two visible tags is a field-test heuristic, not a measured hive tilt/pose.
 * RED Flower IDs 30..33 are opposite the audience; RED Garden 34..37 are audience-side.
 * BLUE Flower IDs 38..41 are audience-side; BLUE Garden 42..45 are opposite the audience.
 * Owns Limelight pipeline 0; do not also start ApriltagDetector/LimelightTracker.
 */
public final class HiveUpTagIDLogic {
    public enum Alliance { RED, BLUE }
    private final Alliance alliance;
    private final int flowerFirstId, gardenFirstId;
    public static final int PIPELINE = 0;
    private static final long MAX_FRAME_AGE_MS = 250;
    private final Limelight3A camera;
    private long lastNewFrameAt;
    private double lastTimestamp = Double.NaN;
    private boolean flowerUp, gardenUp;
    private final HiveTagStability flowerStability = new HiveTagStability();
    private final HiveTagStability gardenStability = new HiveTagStability();
    private final Set<Integer> flowerTags = new HashSet<>();
    private final Set<Integer> gardenTags = new HashSet<>();
    private String status = "Not started";
    // Retain the last sample and rejection after the gate/parking for diagnosis.
    private String diagnostic = "No camera sample yet", lastRejection = "None";
    private String frameState = "Not sampled";
    private long sampledAt, lastDiagnosticLogAt;
    private boolean pollingStopped;
    public void addDiagnostics(Telemetry telemetry) {
        telemetry.addData("LL last sample", diagnostic);
        telemetry.addData("LL frame state", frameState);
        telemetry.addData("LL last rejection", lastRejection);
        telemetry.addData("LL sample age / stopped", sampledAt == 0 ? "Not sampled"
                : String.format(Locale.US, "%.1f s / %s", (System.nanoTime() - sampledAt) / 1e9, pollingStopped));
    }
    private void recordDiagnostic(long now, String state, boolean rejected) {
        frameState = state;
        if (rejected) lastRejection = state + " | " + diagnostic;
        // Once per second in Robot Controller logcat, not every poll.
        if (lastDiagnosticLogAt == 0 || now - lastDiagnosticLogAt >= 1_000_000_000L) {
            RobotLog.ii("HiveVision", "%s | %s | %s", alliance, state, diagnostic);
            lastDiagnosticLogAt = now;
        }
    }

    public HiveUpTagIDLogic(HardwareMap hardwareMap) {
        this(hardwareMap, Alliance.RED);
    }
    /** Select one alliance; reject every tag outside its two four-tag clusters. */
    public HiveUpTagIDLogic(HardwareMap hardwareMap, Alliance alliance) {
        if (alliance == null) throw new IllegalArgumentException("Alliance is required");
        this.alliance = alliance;
        flowerFirstId = alliance == Alliance.RED ? 30 : 38;
        gardenFirstId = alliance == Alliance.RED ? 34 : 42;
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.setPollRateHz(50);
        camera.pipelineSwitch(PIPELINE);
    }
    public void start() { pollingStopped = false; reset(); camera.pipelineSwitch(PIPELINE); camera.start(); }
    /** Reset at FlwrPos1 so INIT/travel observations cannot satisfy the shooting gate. */
    public void reset() {
        clearEvidence();
        lastTimestamp = Double.NaN;
        lastNewFrameAt = 0;
        status = "Waiting for fresh " + alliance + " tags";
    }
    private void clearEvidence() {
        flowerTags.clear(); gardenTags.clear();
        flowerUp = gardenUp = false;
        flowerStability.reset(); gardenStability.reset();
    }
    public void update() {
        long now = System.nanoTime();
        LLResult result = camera.getLatestResult();
        boolean connected = camera.isConnected(), running = camera.isRunning();
        sampledAt = now;
        diagnostic = String.format(Locale.US,
                "connected=%s running=%s expected=%d actual=%s ageMs=%s valid=%s timestamp=%s",
                connected, running, PIPELINE,
                result == null ? "N/A" : String.valueOf(result.getPipelineIndex()),
                result == null ? "N/A" : String.valueOf(result.getStaleness()),
                result == null ? "N/A" : String.valueOf(result.isValid()),
                result == null ? "N/A" : String.valueOf(result.getTimestamp()));
        String rejection = !connected ? "DISCONNECTED"
                : !running ? "POLLING_NOT_RUNNING"
                : result == null ? "NO_RESULT"
                : result.getPipelineIndex() != PIPELINE ? "WRONG_PIPELINE"
                : result.getStaleness() > MAX_FRAME_AGE_MS ? "STALE_FRAME (>250 ms)"
                : !Double.isFinite(result.getTimestamp()) ? "INVALID_TIMESTAMP" : null;
        if (rejection != null) {
            clearEvidence();
            status = rejection;
            recordDiagnostic(now, rejection, true);
            return;
        }
        // Reject frozen camera data instead of treating it as a new observation.
        if (Double.compare(lastTimestamp, result.getTimestamp()) == 0) {
            if (now - lastNewFrameAt > MAX_FRAME_AGE_MS * 1_000_000L) {
                clearEvidence();
                status = "FROZEN_FRAME (>250 ms without timestamp change)";
            }
            boolean frozen = now - lastNewFrameAt > MAX_FRAME_AGE_MS * 1_000_000L;
            recordDiagnostic(now, frozen ? status : "Waiting for next frame (within freshness limit)", frozen);
            return;
        }
        if (lastNewFrameAt != 0 && now - lastNewFrameAt > MAX_FRAME_AGE_MS * 1_000_000L)
            clearEvidence();
        lastTimestamp = result.getTimestamp();
        lastNewFrameAt = now;
        flowerTags.clear(); gardenTags.clear();
        // Fresh no-target results count as brief detection loss, not camera failure.
        if (result.isValid() && result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id >= flowerFirstId && id <= flowerFirstId + 3) flowerTags.add(id);
                else if (id >= gardenFirstId && id <= gardenFirstId + 3) gardenTags.add(id);
                // All other IDs are rejected; duplicate IDs count only once per frame.
            }
        }
        // Camera mounting makes each side independent; other-side tags do not veto detection.
        recordDiagnostic(now, result.isValid() ? "FRESH_FRAME" : "FRESH_NO_VALID_TARGET (count dropout)", false);
        flowerUp = flowerTags.size() >= 2;
        gardenUp = gardenTags.size() >= 2;
        flowerStability.observe(now, flowerUp, HIVE_TAG_DROPOUT_MS * 1_000_000L);
        gardenStability.observe(now, gardenUp, HIVE_TAG_DROPOUT_MS * 1_000_000L);
        status = flowerUp ? alliance + " Flower >=2 tags: rising/up candidate" : "Waiting for >=2 " + alliance + " Flower tags";
    }
    /** Visibility heuristic only; caller supplies a settling delay before shooting. */
    public boolean isRedFlowerUp() { return alliance == Alliance.RED && flowerUp; }
    public boolean isRedGardenUp() { return alliance == Alliance.RED && gardenUp; }
    public boolean isBlueFlowerUp() { return alliance == Alliance.BLUE && flowerUp; }
    public boolean isBlueGardenUp() { return alliance == Alliance.BLUE && gardenUp; }
    public double getFlowerStableSeconds() { return flowerStability.seconds(); }
    public double getGardenStableSeconds() { return gardenStability.seconds(); }
    public boolean isFlowerStable(double seconds) { return flowerUp && flowerStability.ready(seconds); }
    public boolean isGardenStable(double seconds) { return gardenUp && gardenStability.ready(seconds); }
    public String getFlowerTags() { return flowerTags.toString(); }
    public String getGardenTags() { return gardenTags.toString(); }
    public String getStatus() { return status; }
    public void stop() { RobotLog.ii("HiveVision", "Stopped; last state=%s; last rejection=%s", frameState, lastRejection); pollingStopped = true; camera.stop(); clearEvidence(); status = "Stopped"; }
}