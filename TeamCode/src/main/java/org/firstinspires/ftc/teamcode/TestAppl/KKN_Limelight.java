/* ============================================================================
 * Class: KKN_Limelight
 * What it does:
 *   Runs the older Limelight and drivetrain test with vision telemetry and CSV logging support.
 * Applications using it:
 *   KKN_Limelight (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TestAppl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.test.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.test.LimelightTracker;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.test.AprilTagCsvLogger;
import java.io.File;

@TeleOp(name="Krak_N_Auto-2.1")
public class KKN_Limelight extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    int loopCount = 0;

    MecanumDrive driveTest;

    NormalizedColorSensor hardwareColorSensor;
    ColorSensor colorSensor;

    LimelightTracker limelightTracker;
    private final AprilTagCsvLogger tagLogger = new AprilTagCsvLogger();
    private boolean lastLeftBumper,lastRightBumper;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(1000);
        telemetry.addData("INIT"," ****** KrakN WAKING UP ****** ");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLDC");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRDC");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RLDC");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RRDC");

        /*
         * MecanumDrive also configures these motors, so use MecanumDrive
         * as the final drivetrain owner.
         */
        driveTest = new MecanumDrive();
        driveTest.init(hardwareMap, false);

       // hardwareColorSensor =
        //        hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

       // colorSensor =
          //      new ColorSensor(hardwareColorSensor, telemetry);

        // Hardware configuration name must be exactly "limelight".
        limelightTracker =
                new LimelightTracker(hardwareMap, telemetry);

        telemetry.addLine("A = follow YELLOW POLLEN (pipeline 2)");
        telemetry.addLine("Hold Y = yellow telemetry only, motors stopped");
        telemetry.addLine("B = follow RED NECTAR (pipeline 8)");
        telemetry.addLine("X = AprilTag pose telemetry + manual drive");
        telemetry.addLine("CSV: LB=start moving UP, RB=start moving DOWN; D-pad UP/DOWN=end and save");
        telemetry.addData("AprilTag pipeline", LimelightTracker.APRILTAG_PIPELINE);
        telemetry.addLine("Release A/B = manual drive");
        telemetry.update();
    }

    /***********************************************************
     * Test to check Mecanum Drive library
     ***********************************************************/
    public void simpleMecanumWheelTest() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driveTest.drive(forward, strafe, rotate);

        telemetry.addData("Drive Mode", "MANUAL");
        telemetry.addData("FL Power", frontLeftMotor.getPower());
        telemetry.addData("FR Power", frontRightMotor.getPower());
        telemetry.addData("RL Power", rearLeftMotor.getPower());
        telemetry.addData("RR Power", rearRightMotor.getPower());
    }

    /***********************************************************
     * Test color sensor
     ***********************************************************/
    public void simpleColorDetection() {
      //  boolean detected = colorSensor.isObjectDetected();
       // telemetry.addData("Color Sensor Detected", detected);
    }

    /***********************************************************
     * Limelight ball-follow test
     *
     * Hold A -> follow yellow Pollen
     * Hold B -> follow red Nectar
     * Press X -> AprilTag telemetry with manual drive
     * No A/B -> normal gamepad Mecanum control
     ***********************************************************/
    public void simpleLimelightFollowTest() {

        if (gamepad1.y) {
            // Diagnostic mode takes priority over all motion buttons and sticks.
            driveTest.drive(0.0, 0.0, 0.0);
            limelightTracker.setTargetType(LimelightTracker.TargetType.YELLOW_POLLEN);
            telemetry.addData("Drive Mode", "YELLOW TELEMETRY ONLY - STOPPED");
            limelightTracker.update();
        }
        else if (gamepad1.x) {
            // Remains in AprilTag mode after X is released, until A or B is pressed.
            limelightTracker.setTargetType(LimelightTracker.TargetType.APRILTAG);
            simpleMecanumWheelTest();
            limelightTracker.update();
        }
        else if (gamepad1.a) {
            telemetry.addData("Drive Mode", "FOLLOW YELLOW POLLEN");

            limelightTracker.setTargetType(
                    LimelightTracker.TargetType.YELLOW_POLLEN);

            limelightTracker.followTarget(driveTest);
        }
        else if (gamepad1.b) {
            telemetry.addData("Drive Mode", "FOLLOW RED NECTAR");
            limelightTracker.setTargetType(LimelightTracker.TargetType.RED_NECTAR);
            limelightTracker.followTarget(driveTest);
        }
        else {
            // IMPORTANT:
            // Do not also run another drive command after followTarget().
            // When A/B are released, manual driving owns the drivetrain.
            simpleMecanumWheelTest();

            // Still read Limelight so telemetry shows what the camera sees.
            limelightTracker.update();
        }
    }

    @Override
    public void start() {
        telemetry.addData(
                "MACNM",
                " ****** KrakN STARTING UP ****** ");

        limelightTracker.start();
        limelightTracker.setAprilTagLogger(tagLogger);
        lastLeftBumper=gamepad1.left_bumper;
        lastRightBumper=gamepad1.right_bumper;
    }

    @Override
    public void loop() {

        /*
         * This method decides whether the drivetrain is under
         * Limelight control or gamepad control.
         */
        // One CSV per transition. Bumper edges prevent reopening while held.
        boolean startUp=gamepad1.left_bumper && !lastLeftBumper;
        boolean startDown=gamepad1.right_bumper && !lastRightBumper;
        lastLeftBumper=gamepad1.left_bumper;
        lastRightBumper=gamepad1.right_bumper;
        if (!tagLogger.isRecording() && !gamepad1.dpad_up && !gamepad1.dpad_down
                && (startUp || startDown)) {
            tagLogger.open(new File("/sdcard/FIRST/logs/apriltag"));
            tagLogger.mark(startUp ? "MOVING_UP" : "MOVING_DOWN");
            limelightTracker.setTargetType(LimelightTracker.TargetType.APRILTAG);
        }
        if (tagLogger.isRecording()) {
            // Keep the tag pipeline selected throughout capture; Y still stops driving.
            if (gamepad1.y) driveTest.drive(0,0,0);
            else simpleMecanumWheelTest();
            if (gamepad1.dpad_up) tagLogger.setLabel("UP");
            else if (gamepad1.dpad_down) tagLogger.setLabel("DOWN");
            limelightTracker.update();
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                tagLogger.mark(gamepad1.dpad_up ? "UP" : "DOWN");
                tagLogger.close(); // Flush immediately; leave the OpMode running.
            }
        } else simpleLimelightFollowTest();
        telemetry.addData("CSV", tagLogger.getStatus());
        telemetry.addData("CSV file", tagLogger.getPath());
        telemetry.addData("Hive label (manual)", tagLogger.getLabel());

        // Existing color-sensor test can continue at the same time.
       // simpleColorDetection();

        telemetry.addData("Loop Count", loopCount++);
        telemetry.update();
    }

    @Override
    public void stop() {
        tagLogger.close(); // Flush buffered samples even before shutting down hardware.
        telemetry.addData(
                "MACNM",
                " ****** KrakN SHUTTING DOWN ****** ");

        driveTest.drive(0.0, 0.0, 0.0);
        limelightTracker.stop();
       // colorSensor.stopColorSensor();
        telemetry.update();
    }
}
