/* ============================================================================
 * Class: HiveAprilTagCalibration
 * What it does:
 *   Tests CSV file creation by default; optionally records camera-only hive calibration data.
 * Applications using it:
 *   HiveAprilTagCalibration (Driver Station entry point).
 *   Calibration currently has LOG_FILE_TEST_ONLY=true: camera access is skipped.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TestAppl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.test.AprilTagCsvLogger;
import org.firstinspires.ftc.teamcode.subsystems.test.ApriltagDetector;
import java.io.File;

/** Currently defaults to a hardware-free INIT file-write test (LOG_FILE_TEST_ONLY).
 * With that flag false, camera-only calibration: no drivetrain, intake, or shooter hardware is initialized.
 * Select hive in INIT or during recording: A=BHF, B=BHG, X=RHF, Y=RHG.
 * START opens a CSV. Dpad up/down labels observed UP/DOWN; bumpers label motion.
 * STOP flushes/closes. Labels are operator observations, NOT classifier predictions.
 * All visible tags are logged (including other hives) to diagnose mistaken tag groups.
 */
@TeleOp(name="Hive AprilTag Calibration", group="Calibration")
public class HiveAprilTagCalibration extends OpMode {
    // Temporary storage test: set false to restore camera calibration.
    private static final boolean LOG_FILE_TEST_ONLY = true;
    private ApriltagDetector detector;
    private final AprilTagCsvLogger logger=new AprilTagCsvLogger();
    private ApriltagDetector.Hive hive=ApriltagDetector.Hive.BHF;
    private String observed="UNKNOWN";
    @Override public void init() {
        if (LOG_FILE_TEST_ONLY) {
            // No hardwareMap access. Close immediately so the test row is on disk
            // even if START is never pressed; preserve logger errors in telemetry.
            logger.open(new File("/sdcard/FIRST/logs/apriltag"));
            logger.mark("INIT_FILE_WRITE_TEST");
            logger.close();
            showFileTest();
            return;
        }
        detector=new ApriltagDetector(hardwareMap,telemetry);
        detector.setAprilTagLogger(logger);
        detector.start();
        telemetry.addLine("Camera only. Select A=BHF B=BHG X=RHF Y=RHG; START records.");
    }
    @Override public void init_loop() { sample(); }
    @Override public void start() {
        if (LOG_FILE_TEST_ONLY) { showFileTest(); return; }
        logger.open(new File("/sdcard/FIRST/logs/apriltag"));
        observed="UNKNOWN";
    }
    @Override public void loop() { sample(); }
    private void sample() {
        if (LOG_FILE_TEST_ONLY) { showFileTest(); return; }
        ApriltagDetector.Hive previousHive=hive;
        if(gamepad1.a) hive=ApriltagDetector.Hive.BHF;
        else if(gamepad1.b) hive=ApriltagDetector.Hive.BHG;
        else if(gamepad1.x) hive=ApriltagDetector.Hive.RHF;
        else if(gamepad1.y) hive=ApriltagDetector.Hive.RHG;
        if(hive!=previousHive) observed="UNKNOWN";
        if(gamepad1.dpad_up) observed="UP";
        else if(gamepad1.dpad_down) observed="DOWN";
        else if(gamepad1.left_bumper) observed="MOVING_UP";
        else if(gamepad1.right_bumper) observed="MOVING_DOWN";
        String label=hive.name()+"_"+observed;
        if(!label.equals(logger.getLabel())) logger.mark(label);
        detector.isHiveUp(hive);
        // isHiveUp logs and displays all tag poses from the same camera frame.
        telemetry.addData("Observed label",label);
        telemetry.addData("CSV",logger.getStatus());
        telemetry.addData("CSV path",logger.getPath());
        telemetry.addLine("A=BHF B=BHG X=RHF Y=RHG | Dpad UP/DOWN | LB/RB moving UP/DOWN");
        telemetry.addLine("Angles are uncalibrated camera-space estimates, not FTC cluster poses.");
    }
    private void showFileTest() {
        telemetry.addLine("FILE TEST ONLY - Limelight hardware disabled");
        telemetry.addData("CSV", logger.getStatus());
        telemetry.addData("CSV path", logger.getPath());
        telemetry.addLine("INIT created and closed the test CSV. START is not required.");
    }
    @Override public void stop() {
        logger.close();
        if(detector!=null) detector.stop();
    }
}