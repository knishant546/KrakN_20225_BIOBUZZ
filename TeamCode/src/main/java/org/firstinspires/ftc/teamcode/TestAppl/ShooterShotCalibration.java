/* ============================================================================
 * Class: ShooterShotCalibration
 * What it does: Runs BioBuzzTeleop controls and captures manual shot samples to CSV.
 * Applications using it: Shooter Shot Calibration (standalone Driver Station OpMode).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TestAppl;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.TeleopAppl.BioBuzzTeleop;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooter;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooter;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import java.io.*;
import java.util.*;

/** A captures current readings after an operator-observed success, not ball-release data.
 * One row per new Gamepad 1 A press. No automatic aiming, firing or hive-up decision.
 * Missing sensors do not prevent capture: CSV status and blank fields describe missing data.
 * Camera poses are raw tag-relative measurements, not the hive opening or shooter distance.
 */
@Disabled
@TeleOp(name="Shooter Shot Calibration", group="Calibration")
public class ShooterShotCalibration extends BioBuzzTeleop {
    public static final int APRILTAG_PIPELINE = 0;
    public static final long MAX_FRAME_AGE_MS = 250;
    private PinpointLocalizer localizer;
    private Limelight3A camera;
    private BufferedWriter writer;
    private String path="", logStatus="Not opened", poseStatus="Unavailable", cameraStatus="Unavailable";
    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;
    private boolean allianceLocked;
    private boolean aWasPressed;
    private long started;
    private int samples;

    @Override public void init() {
        super.init(); // Same motor/arm mappings, powers, directions and controls as TeleOp.
        try {
            localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
            poseStatus = "Available (raw odometry; not a health guarantee)";
        } catch (RuntimeException e) { poseStatus="Unavailable: "+e.getMessage(); }
        try {
            camera=hardwareMap.get(Limelight3A.class,"limelight");
            camera.pipelineSwitch(APRILTAG_PIPELINE);
            camera.start();
            cameraStatus="Waiting for pipeline 0";
        } catch (RuntimeException e) { cameraStatus="Unavailable: "+e.getMessage(); }
        openCsv();
        telemetry.setMsTransmissionInterval(100);
        showLogging(null);
    }

    @Override public void init_loop() {
        super.init_loop();
        // Select only during INIT; simultaneous bumpers leave the choice unchanged.
        if(gamepad1.left_bumper && !gamepad1.right_bumper) alliance=Alliance.RED;
        else if(gamepad1.right_bumper && !gamepad1.left_bumper) alliance=Alliance.BLUE;
        showLogging(readPose());
    }

    @Override public void start() {
        allianceLocked=true;
        super.start();
        started=System.nanoTime();
        aWasPressed=gamepad1.a;
        if(localizer!=null) {
            try { localizer.setPose(new Pose(0,0,0)); }
            catch(RuntimeException e) { poseStatus="Reset failed: "+e.getMessage(); localizer=null; }
        }
    }

    @Override public void loop() {
        super.loop();
        Pose pose=readPose();
        LLResult result=null;
        try {
            if(camera!=null) result=camera.getLatestResult();
            if(camera!=null) cameraStatus=result==null?"NO_RESULT":
                result.getPipelineIndex()!=APRILTAG_PIPELINE?"WRONG_PIPELINE":
                result.getStaleness()>MAX_FRAME_AGE_MS?"STALE":
                !result.isValid()?"NO_VALID_TARGET":"FRESH";
        } catch(RuntimeException e) { cameraStatus="READ_ERROR: "+e.getMessage(); }
        if("FRESH".equals(cameraStatus)) {
            boolean hasAllianceTag=false;
            if(result.getFiducialResults()!=null)
                for(LLResultTypes.FiducialResult tag:result.getFiducialResults())
                    if(acceptTag(tag.getFiducialId())) { hasAllianceTag=true; break; }
            if(!hasAllianceTag) cameraStatus="NO_ALLIANCE_TAG";
        }
        if(gamepad1.a && !aWasPressed && !gamepad1.start && !gamepad1.back)
            capture(pose,result);
        aWasPressed=gamepad1.a;
        showLogging(pose);
        if(result!=null) telemetry.addData("Frame age (ms)",result.getStaleness());
        if("FRESH".equals(cameraStatus) && result!=null && result.getFiducialResults()!=null)
            for(LLResultTypes.FiducialResult tag:result.getFiducialResults()) {
                if(!acceptTag(tag.getFiducialId())) continue;
                Pose3D target=tag.getTargetPoseCameraSpace();
                Position pos=target==null||target.getPosition()==null?null:
                        target.getPosition().toUnit(DistanceUnit.METER);
                telemetry.addData("Tag "+tag.getFiducialId()+" tx/ty (deg)","%.2f / %.2f",
                        tag.getTargetXDegrees(),tag.getTargetYDegrees());
                if(pos!=null) telemetry.addData("Tag "+tag.getFiducialId()+" camera XYZ (m)",
                        "%.3f / %.3f / %.3f",pos.x,pos.y,pos.z);
            }
    }

    private Pose readPose() {
        if(localizer==null) return null;
        try { localizer.update(); return localizer.state().pose(); }
        catch(RuntimeException e) { poseStatus="Read failed: "+e.getMessage(); return null; }
    }

    private void openCsv() {
        try {
            File dir=new File("/sdcard/FIRST/logs/shooter-calibration");
            if(!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create directory");
            File file=File.createTempFile("shots-"+System.currentTimeMillis()+"-", ".csv",dir);
            path=file.getAbsolutePath();
            writer=new BufferedWriter(new FileWriter(file));
            List<String> header=new ArrayList<>(Arrays.asList("sample_id","host_unix_ms","elapsed_s",
                "alliance","operator_label","attempts_manual","hits_manual","notes_manual","pose_status",
                "robot_x_in","robot_y_in","robot_heading_deg","selected_speed_percent",
                "target_ticks_s","measured_ticks_s_at_capture","sdk_power_at_capture","battery_v",
                "nectar_selected_speed_percent","nectar_target_ticks_s","nectar_measured_ticks_s","nectar_sdk_power",
                "camera_status","pipeline","frame_timestamp","frame_age_ms","alliance_visible_ids"));
            // Fixed per-tag columns avoid changing the target when a different tag becomes largest.
            for(int id=30;id<=45;id++) for(String field:new String[]{"x_m","y_m","z_m",
                "roll_deg","pitch_deg","yaw_deg","tx_deg","ty_deg"}) header.add("tag"+id+"_"+field);
            writeRow(header);
            logStatus="Ready; header saved";
        } catch(IOException e) { failLog(e); }
    }

    private void capture(Pose pose,LLResult result) {
        if(writer==null) { logStatus="Cannot save: CSV unavailable"; return; }
        PollenShooter shooter=PollenShooter.getInstance();
        NectarShooter nectar=NectarShooter.getInstance();
        double volts=Double.NaN;
        for(VoltageSensor sensor:hardwareMap.voltageSensor) {
            double value=sensor.getVoltage();
            if(value>0 && (Double.isNaN(volts)||value<volts)) volts=value;
        }
        boolean fresh="FRESH".equals(cameraStatus);
        Map<Integer,LLResultTypes.FiducialResult> tags=new HashMap<>();
        List<String> ids=new ArrayList<>();
        if(fresh && result.getFiducialResults()!=null)
            for(LLResultTypes.FiducialResult tag:result.getFiducialResults()) {
                if(!acceptTag(tag.getFiducialId())) continue;
                tags.put(tag.getFiducialId(),tag);
                ids.add(Integer.toString(tag.getFiducialId()));
            }
        List<String> row=new ArrayList<>(Arrays.asList(Integer.toString(samples+1),
            Long.toString(System.currentTimeMillis()),num((System.nanoTime()-started)/1e9),
            alliance.name(),"SUCCESS_REPORTED_AT_BUTTON_PRESS","","","",pose==null?"UNAVAILABLE":poseStatus,
            pose==null?"":num(pose.x()),pose==null?"":num(pose.y()),
            pose==null?"":num(Math.toDegrees(pose.heading())),Integer.toString(getSelectedShooterSpeedPercent()),
            num(shooter.getTargetVelocity()),num(shooter.measuredVelocity()),num(shooter.getShooterPower()),num(volts),
            Integer.toString(getSelectedNectarSpeedPercent()),num(nectar.getTargetVelocity()),
            num(nectar.measuredVelocity()),num(nectar.getShooterPower()),
            cameraStatus,result==null?"":Integer.toString(result.getPipelineIndex()),
            result==null?"":num(result.getTimestamp()),result==null?"":Long.toString(result.getStaleness()),
            String.join(";",ids)));
        for(int id=30;id<=45;id++) {
            LLResultTypes.FiducialResult tag=tags.get(id);
            Pose3D tp=tag==null?null:tag.getTargetPoseCameraSpace();
            Position p=tp==null||tp.getPosition()==null?null:tp.getPosition().toUnit(DistanceUnit.METER);
            YawPitchRollAngles a=tp==null?null:tp.getOrientation();
            boolean valid=p!=null && Double.isFinite(p.x) && Double.isFinite(p.y) && Double.isFinite(p.z)
                    && (p.x!=0 || p.y!=0 || p.z!=0);
            row.add(valid?num(p.x):""); row.add(valid?num(p.y):""); row.add(valid?num(p.z):"");
            row.add(valid&&a!=null?num(a.getRoll(AngleUnit.DEGREES)):"");
            row.add(valid&&a!=null?num(a.getPitch(AngleUnit.DEGREES)):"");
            row.add(valid&&a!=null?num(a.getYaw(AngleUnit.DEGREES)):"");
            row.add(tag==null?"":num(tag.getTargetXDegrees()));
            row.add(tag==null?"":num(tag.getTargetYDegrees()));
        }
        try { writeRow(row); samples++; logStatus="Saved sample "+samples+" ("+alliance+", "+cameraStatus+")"; }
        catch(IOException e) { failLog(e); }
    }

    /** Reject opposing-alliance and non-hive IDs before telemetry and CSV extraction. */
    private boolean acceptTag(int id) {
        return alliance==Alliance.RED ? id>=30 && id<=37 : id>=38 && id<=45;
    }

    private static String num(double value) {
        return Double.isFinite(value)?String.format(Locale.US,"%.6f",value):"";
    }
    private void writeRow(List<String> cells) throws IOException {
        for(int i=0;i<cells.size();i++) {
            if(i>0) writer.write(',');
            writer.write('"'); writer.write(cells.get(i).replace("\"","\"\"")); writer.write('"');
        }
        writer.newLine(); writer.flush(); // Each explicit capture is flushed, not buffered until STOP.
    }
    private void failLog(IOException e) {
        logStatus="CSV ERROR: "+e.getMessage();
        if(writer!=null) try { writer.close(); } catch(IOException ignored) { }
        writer=null;
    }
    private void showLogging(Pose pose) {
        telemetry.addLine("CALIBRATION: Gamepad 1 A saves current readings after a successful shot.");
        telemetry.addLine("Stay at shot position; readings are at capture time, NOT ball release.");
        telemetry.addData("Alliance",alliance+ (allianceLocked?" (locked)":" (INIT selection)"));
        if(!allianceLocked) telemetry.addLine("Gamepad 1 LB = RED (30-37), RB = BLUE (38-45); START locks selection.");
        telemetry.addData("CSV status",logStatus);
        telemetry.addData("CSV path",path);
        telemetry.addData("Saved samples",samples);
        telemetry.addData("Camera",cameraStatus);
        telemetry.addData("Odometry",poseStatus);
        if(pose!=null) telemetry.addData("Capture pose (in, in, deg)","%.2f, %.2f, %.2f",
                pose.x(),pose.y(),Math.toDegrees(pose.heading()));
    }
    @Override public void stop() {
        try { super.stop(); }
        finally {
            try { if(camera!=null) camera.stop(); }
            finally {
                if(writer!=null) try { writer.close(); writer=null; }
                catch(IOException e) { failLog(e); }
            }
        }
    }
}