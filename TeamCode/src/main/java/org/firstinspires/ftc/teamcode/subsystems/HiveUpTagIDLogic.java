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
    private final Set<Integer> flowerTags = new HashSet<>();
    private final Set<Integer> gardenTags = new HashSet<>();
    private String status = "Not started";

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
    public void start() { reset(); camera.pipelineSwitch(PIPELINE); camera.start(); }
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
    }
    public void update() {
        long now = System.nanoTime();
        LLResult result = camera.getLatestResult();
        if (!camera.isConnected() || !camera.isRunning() || result == null
                || !result.isValid() || result.getPipelineIndex() != PIPELINE
                || result.getStaleness() > MAX_FRAME_AGE_MS
                || !Double.isFinite(result.getTimestamp())) {
            clearEvidence();
            status = "No valid fresh pipeline-0 frame";
            return;
        }
        // Reject frozen camera data instead of treating it as a new observation.
        if (Double.compare(lastTimestamp, result.getTimestamp()) == 0) {
            if (now - lastNewFrameAt > MAX_FRAME_AGE_MS * 1_000_000L) {
                clearEvidence();
                status = "Frozen frame; detection cleared";
            }
            return;
        }
        if (lastNewFrameAt != 0 && now - lastNewFrameAt > MAX_FRAME_AGE_MS * 1_000_000L)
            clearEvidence();
        lastTimestamp = result.getTimestamp();
        lastNewFrameAt = now;
        flowerTags.clear(); gardenTags.clear();
        if (result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id >= flowerFirstId && id <= flowerFirstId + 3) flowerTags.add(id);
                else if (id >= gardenFirstId && id <= gardenFirstId + 3) gardenTags.add(id);
                // All other IDs are rejected; duplicate IDs count only once per frame.
            }
        }
        // Camera mounting makes each side independent; other-side tags do not veto detection.
        flowerUp = flowerTags.size() >= 2;
        gardenUp = gardenTags.size() >= 2;
        status = flowerUp ? alliance + " Flower >=2 tags: rising/up candidate" : "Waiting for >=2 " + alliance + " Flower tags";
    }
    /** Visibility heuristic only; caller supplies a settling delay before shooting. */
    public boolean isRedFlowerUp() { return alliance == Alliance.RED && flowerUp; }
    public boolean isRedGardenUp() { return alliance == Alliance.RED && gardenUp; }
    public boolean isBlueFlowerUp() { return alliance == Alliance.BLUE && flowerUp; }
    public boolean isBlueGardenUp() { return alliance == Alliance.BLUE && gardenUp; }
    public String getFlowerTags() { return flowerTags.toString(); }
    public String getGardenTags() { return gardenTags.toString(); }
    public String getStatus() { return status; }
    public void stop() { camera.stop(); clearEvidence(); status = "Stopped"; }
}