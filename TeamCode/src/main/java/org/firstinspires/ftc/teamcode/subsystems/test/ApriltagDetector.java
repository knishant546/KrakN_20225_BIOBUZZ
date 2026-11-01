/* ============================================================================
 * Class: ApriltagDetector
 * What it does:
 *   Reads Limelight AprilTags and estimates hive state from configurable pose-angle references.
 * Applications using it:
 *   HiveAprilTagCalibration
 *   Includes indirect/source dependencies; optional features may be inactive.
 *   Calibration currently has LOG_FILE_TEST_ONLY=true: camera access is skipped.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.List;

/** Camera-only AprilTag/hive reader shared by calibration and future autonomous OpModes.
 * Hardware name: limelight. Configure pipeline 1 for 36h11 and Full 3D Targeting.
 * Construct in INIT, start polling with start(), query isHiveUp() each loop/before
 * feeding a ball, and call stop() from the OpMode's stop/finally handler.
 * Do not run a second Limelight owner (such as LimelightTracker) simultaneously.
 * The optional CSV logger is owned and closed by the calling OpMode.
 * No pollen detection, drivetrain, intake, or shooter dependencies.
 */
public final class ApriltagDetector {
    public static final int APRILTAG_PIPELINE=1;
    private static final long MAX_RESULT_AGE_MS=250;
    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private AprilTagCsvLogger aprilTagLogger;

    public ApriltagDetector(HardwareMap hardwareMap, Telemetry telemetry) {
        this.limelight=hardwareMap.get(Limelight3A.class,"limelight");
        this.telemetry=telemetry;
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
    }
    public void start() { limelight.pipelineSwitch(APRILTAG_PIPELINE); limelight.start(); }
    public void stop() { limelight.stop(); hiveStatus="UNKNOWN: stopped"; }
    public void setAprilTagLogger(AprilTagCsvLogger logger) { aprilTagLogger=logger; }
    public boolean isConnected() { return limelight.isConnected(); }
    public boolean isRunning() { return limelight.isRunning(); }

    /** Application hive IDs, NOT physical AprilTag IDs or FTC SDK cluster IDs. */
    public enum Hive {
        BHF(11, 38), BHG(12, 42), RHF(13, 30), RHG(14, 34);
        public final int id;
        public final int firstTag;
        Hive(int id, int firstTag) { this.id=id; this.firstTag=firstTag; }
        public boolean contains(int tagId) { return tagId>=firstTag && tagId<firstTag+4; }
        public static Hive fromId(int id) {
            for (Hive hive:values()) if(hive.id==id) return hive;
            throw new IllegalArgumentException("Unknown hive ID: "+id);
        }
    }

    // Field convention: BHF=Blue Audience, BHG=Blue Scoring,
    // RHF=Red Scoring, RHG=Red Audience. Each group has four unique tags.
    // FTC guidance: abs(CLUSTER ftcPose.roll)<90 means potentially scorable.
    // https://ftc-docs.firstinspires.org/apriltag-clusters
    // Limelight individual-tag camera roll is NOT that FTC cluster pose.
    // These zero references and the 90-degree boundary are UNCALIBRATED estimates,
    // explicitly requested for initial testing at four fixed shooting positions.
    // Use the separate calibration logger to verify axis, sign, and per-tag reference
    // before using this result to feed balls. No cluster pose fusion is performed.
    public static final double[] UPRIGHT_ROLL_DEG_BY_TAG = new double[16]; // IDs 30..45
    private String hiveStatus="UNKNOWN";
    public String getHiveStatus() { return hiveStatus; }

    /** True only when at least one matching tag has valid UP pose and none disagree.
     * False includes DOWN, transitional, unavailable, stale, and conflicting data.
     * Call start() once, then call this every control loop / before each shot.
     */
    public boolean isHiveUp(int hiveId) { return isHiveUp(Hive.fromId(hiveId)); }
    public boolean isHiveUp(Hive hive) {
        if(hive==null) throw new IllegalArgumentException("Hive is required");
        // This detector owns pipeline 1 for its entire lifecycle.
        LLResult result=getFreshResult();
        if(aprilTagLogger!=null) aprilTagLogger.record(result);
        if(result!=null) addAprilTagTelemetry(result);
        hiveStatus="UNKNOWN: no fresh matching pose";
        int up=0, down=0, invalid=0;
        if(result!=null && result.getFiducialResults()!=null) {
            for(LLResultTypes.FiducialResult tag:result.getFiducialResults()) {
                int id=tag.getFiducialId();
                if(!hive.contains(id)) continue;
                Pose3D pose=tag.getTargetPoseCameraSpace();
                Position p=pose==null?null:pose.getPosition();
                YawPitchRollAngles a=pose==null?null:pose.getOrientation();
                double roll=a==null?Double.NaN:a.getRoll(AngleUnit.DEGREES);
                if(p==null || !Double.isFinite(p.x) || !Double.isFinite(p.y)
                        || !Double.isFinite(p.z) || (p.x==0 && p.y==0 && p.z==0)
                        || !Double.isFinite(roll)) { invalid++; continue; }
                double error=AngleUnit.normalizeDegrees(roll-UPRIGHT_ROLL_DEG_BY_TAG[id-30]);
                telemetry.addData("Hive tag "+id+" roll / relative", "%.1f / %.1f",roll,error);
                if(Math.abs(error)<90) up++; else down++;
            }
        }
        if(invalid>0) hiveStatus="UNKNOWN: invalid matching pose";
        else if(up>0 && down>0) hiveStatus="UNKNOWN: conflicting tags";
        else if(up>0) hiveStatus="UP (uncalibrated estimate)";
        else if(down>0) hiveStatus="DOWN (uncalibrated estimate)";
        telemetry.addData("Hive",hive+" ("+hive.id+")");
        telemetry.addData("Hive state",hiveStatus);
        return up>0 && down==0 && invalid==0;
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
        if (result.getPipelineIndex() != APRILTAG_PIPELINE) {
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

}
