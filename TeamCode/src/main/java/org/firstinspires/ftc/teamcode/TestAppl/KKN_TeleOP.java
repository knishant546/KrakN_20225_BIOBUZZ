package org.firstinspires.ftc.teamcode.TestAppl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;



//@Disabled
@TeleOp(name="KKN_Teleop")
//@Autonomous(name="Krak_N_Teleop-2")
public class KKN_TeleOP extends OpMode {
     DcMotor frontLeftMotor;
     DcMotor frontRightMotor;
     DcMotor rearLeftMotor;
     DcMotor rearRightMotor;
     int loopCount = 0;
     MecanumDrive driveTest;
    Intake intake;
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
        driveTest = new MecanumDrive();
        driveTest.init(hardwareMap,false);
        driveTest.drive(0,0,0);
       // hardwareColorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
      //  colorSensor = new ColorSensor(hardwareColorSensor, telemetry);
       intake= new Intake();
       intake.init(hardwareMap);

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
        telemetry.addData("RL Power", rearLeftMotor.getPower());
        telemetry.addData("RR Power", rearRightMotor.getPower());
        //telemetry.addData("Loop Count", loopCount++);

    }

    public void simpleIntake(){
        if (gamepad1.dpad_up) {
            intake.start();
        }
        else if (gamepad1.dpad_down) {
            intake.stop();
        }

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

        simpleMacenumWheelTest();
        // simpleColorDetection();
         simpleIntake();

    }
    public void start() {
        telemetry.addData("MACNM"," ****** KrakN STARTING UP****** ");
    }

    public void stop() {
        telemetry.addData("MACNM"," ****** KrakN SHUTTING DOWN ****** ");
        driveTest.drive(0,0,0);
       //   colorSensor.stopColorSensor();
       intake.stop();

    }


    }
