package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.ColorSensor;


//@Disabled
@TeleOp(name="KKN_TEST")
//@Autonomous(name="Krak_N_Teleop-2")
public class KKN_TEST extends OpMode {
     DcMotor frontLeftMotor;
     DcMotor frontRightMotor;
     DcMotor rearLeftMotor;
     DcMotor rearRightMotor;
     int loopCount = 0;
     MecanumDrive driveTest;
     double forward, strafe, rotate;
   // private NormalizedColorSensor colorSensor;
     NormalizedColorSensor hardwareColorSensor;
     ColorSensor colorSensor;


    @Override
    public void init() {
         telemetry.addData("INIT"," ****** KrakN WAKING UP ****** ");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FLDC");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRDC");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RLDC");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RRDC");
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        driveTest = new MecanumDrive();
        driveTest.init(hardwareMap,false);
        forward = -gamepad1.left_stick_y; // Forward backward on Y axis
        strafe = gamepad1.left_stick_x; //sideways
        rotate = gamepad1.right_stick_x; //rotate

         hardwareColorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
         colorSensor = new ColorSensor(hardwareColorSensor, telemetry);

        }
/**********************************************************
        Test to check if we are able to rotate wheels #1
/**********************************************************/

        public void simpleWheelRotation() {
        if (gamepad1.dpad_up) {
            frontLeftMotor.setPower(0.5);
            rearLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            rearRightMotor.setPower(0.5);
        }
        if (gamepad1.dpad_down) {
            frontLeftMotor.setPower(0.0);
            rearLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            rearRightMotor.setPower(0.0);
    }
        telemetry.addData("FL Power", frontLeftMotor.getPower());
        telemetry.addData("FR Power", frontRightMotor.getPower());
        telemetry.addData("Loop Count", loopCount++);
        }

    /***********************************************************
     Test to check if we are able to use Mecanum Drive library #2
     /**********************************************************/
    public void simpleMacenumWheelTest() {
        telemetry.addData("MACNM"," ****** KrakN ROLLING ****** ");
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driveTest.drive(forward, strafe, rotate);
        telemetry.addData("FL Power", frontLeftMotor.getPower());
        telemetry.addData("FR Power", frontRightMotor.getPower());
        telemetry.addData("FL Power", rearLeftMotor.getPower());
        telemetry.addData("FR Power", rearRightMotor.getPower());
        telemetry.addData("Loop Count", loopCount++);

    }

    /***********************************************************
     Test to check if we are able to use Color Sensor #3
     /**********************************************************/
    public void simpleColorDetection(){
        boolean detected = colorSensor.isObjectDetected();
        if(detected){

        telemetry.addData("Detected", detected);
        }
        telemetry.update();

    }
    public void loop(){
         //simpleWheelRotation();
         simpleMacenumWheelTest();
         simpleColorDetection();

    }
    public void start() {
        telemetry.addData("MACNM"," ****** KrakN STARTING UP****** ");
    }

    public void stop() {
        telemetry.addData("MACNM"," ****** KrakN SHUTTING DOWN ****** ");
        //driveTest.drive(0,0,0);
        colorSensor.stopColorSensor();
    }


    }
