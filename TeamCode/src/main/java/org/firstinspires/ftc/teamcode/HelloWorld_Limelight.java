package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.mechanism.ColorSensor;
import org.firstinspires.ftc.teamcode.mechanism.LimelightTracker;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name="Krak_N_Auto-2.1")
public class HelloWorld_Limelight extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    int loopCount = 0;

    MecanumDrive driveTest;

    NormalizedColorSensor hardwareColorSensor;
    ColorSensor colorSensor;

    LimelightTracker limelightTracker;

    @Override
    public void init() {
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

        hardwareColorSensor =
                hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensor =
                new ColorSensor(hardwareColorSensor, telemetry);

        // Hardware configuration name must be exactly "limelight".
        limelightTracker =
                new LimelightTracker(hardwareMap, telemetry);

        telemetry.addLine("A = follow YELLOW POLLEN");
        telemetry.addLine("B = follow RED NECTAR");
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
        boolean detected = colorSensor.isObjectDetected();
        telemetry.addData("Color Sensor Detected", detected);
    }

    /***********************************************************
     * Limelight ball-follow test
     *
     * Hold A -> follow yellow Pollen
     * Hold B -> follow red Nectar
     * No A/B -> normal gamepad Mecanum control
     ***********************************************************/
    public void simpleLimelightFollowTest() {

        if (gamepad1.a) {
            telemetry.addData("Drive Mode", "FOLLOW YELLOW POLLEN");

            limelightTracker.setTargetType(
                    LimelightTracker.TargetType.YELLOW_POLLEN);

            limelightTracker.followTarget(driveTest);
        }
        else if (gamepad1.b) {
            telemetry.addData("Drive Mode", "FOLLOW RED NECTAR");

            limelightTracker.setTargetType(
                    LimelightTracker.TargetType.RED_NECTAR);

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
    }

    @Override
    public void loop() {

        /*
         * This method decides whether the drivetrain is under
         * Limelight control or gamepad control.
         */
        simpleLimelightFollowTest();

        // Existing color-sensor test can continue at the same time.
        simpleColorDetection();

        telemetry.addData("Loop Count", loopCount++);
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData(
                "MACNM",
                " ****** KrakN SHUTTING DOWN ****** ");

        driveTest.drive(0.0, 0.0, 0.0);
        limelightTracker.stop();
        colorSensor.stopColorSensor();
        telemetry.update();
    }
}
