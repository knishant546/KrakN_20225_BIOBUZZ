/* ============================================================================
 * Class: AprilTagCsvLogger
 * What it does:
 *   Writes AprilTag calibration samples and operator markers to CSV files.
 * Applications using it:
 *   HiveAprilTagCalibration
 *   KKN_Limelight
 *   Includes indirect/source dependencies; optional features may be inactive.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.io.*;
import java.util.Locale;

/** Buffered calibration recording; labels describe the operator's observation, not a prediction. */
public final class AprilTagCsvLogger {
    private BufferedWriter writer;
    private long started, flushed, lastMissing;
    private double lastFrame=Double.NaN;
    private String label="UNKNOWN",status="Not started",path="";
    private long rows;

    public void open(File directory) {
        close();
        try {
            if(!directory.isDirectory() && !directory.mkdirs())throw new IOException("Cannot create log folder");
            File file=File.createTempFile("apriltag-"+System.currentTimeMillis()+"-", ".csv",directory);
            path=file.getAbsolutePath();
            writer=new BufferedWriter(new FileWriter(file));
            writer.write("host_unix_ms,elapsed_ms,label,status,frame_timestamp,pipeline,age_ms,tag_id,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg,tx_deg,ty_deg\n");
            writer.flush(); started=flushed=System.nanoTime(); lastMissing=0;
            lastFrame=Double.NaN; rows=0; label="UNKNOWN"; status="Recording";
        } catch(IOException e) { fail(e); }
    }
    public void setLabel(String value) { label=value; }
    public boolean isRecording() { return writer!=null; }
    /** Operator event, not a measured pose or automatic hive classification. */
    public void mark(String value) {
        label=value;
        if(writer!=null)try { row(System.nanoTime(),"MANUAL_MARKER",",,,,,,,,,,,"); }
        catch(IOException e) { fail(e); }
    }
    public String getLabel() { return label; }
    public String getStatus() { return status+"; rows="+rows; }
    public String getPath() { return path; }

    /** Called only in AprilTag mode with the same fresh result used by telemetry. */
    public void record(LLResult result) {
        if(writer==null)return;
        long now=System.nanoTime();
        try {
            if(result==null) {
                // Record loss of valid/fresh data at most four times per second.
                if(now-lastMissing>=250000000L) {
                    row(now,"NO_FRESH_RESULT",",,,,,,,,,,,"); lastMissing=now;
                }
            } else if(Double.compare(result.getTimestamp(),lastFrame)!=0) {
                lastFrame=result.getTimestamp();
                String frame=String.format(Locale.US,"%.3f,%d,%.1f,",result.getTimestamp(),
                        result.getPipelineIndex(),(double)result.getStaleness());
                if(result.getFiducialResults()==null || result.getFiducialResults().isEmpty())
                    row(now,"NO_TAGS",frame+",,,,,,,,");
                else for(LLResultTypes.FiducialResult tag:result.getFiducialResults()) {
                    Pose3D pose=tag.getTargetPoseCameraSpace();
                    Position p=pose==null || pose.getPosition()==null?null:pose.getPosition().toUnit(DistanceUnit.METER);
                    YawPitchRollAngles a=pose==null?null:pose.getOrientation();
                    double roll=a==null?Double.NaN:a.getRoll(AngleUnit.DEGREES);
                    double pitch=a==null?Double.NaN:a.getPitch(AngleUnit.DEGREES);
                    double yaw=a==null?Double.NaN:a.getYaw(AngleUnit.DEGREES);
                    boolean valid=p!=null && Double.isFinite(p.x)&&Double.isFinite(p.y)&&Double.isFinite(p.z)
                            &&(p.x!=0||p.y!=0||p.z!=0)&&Double.isFinite(roll)&&Double.isFinite(pitch)&&Double.isFinite(yaw);
                    String values=String.format(Locale.US,"%d,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f",
                            tag.getFiducialId(),p==null?Double.NaN:p.x,p==null?Double.NaN:p.y,
                            p==null?Double.NaN:p.z,roll,pitch,yaw,tag.getTargetXDegrees(),tag.getTargetYDegrees());
                    row(now,valid?"POSE":"POSE_UNAVAILABLE",frame+values);
                }
            }
            if(now-flushed>=1000000000L) { writer.flush(); flushed=now; }
        } catch(IOException e) { fail(e); }
    }
    private void row(long now,String state,String data) throws IOException {
        writer.write(System.currentTimeMillis()+","+(now-started)/1000000+","+label+","+state+","+data+"\n");
        rows++;
    }
    private void fail(IOException error) {
        status="LOG ERROR: "+error.getMessage();
        if(writer!=null)try { writer.close(); } catch(IOException ignored) { }
        writer=null;
    }
    public void close() {
        if(writer==null)return;
        try { writer.close(); writer=null; status="Saved"; } catch(IOException e) { fail(e); }
    }
}
