package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import com.pedropathing.revhub.localizers.PinpointConfig;

public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    GoBildaPinpointDriver pinpoint;

    private boolean isPinpoint;

    public void init(HardwareMap hwMap, boolean isGobildaPinpointIMU) {
        frontLeftMotor = hwMap.get(DcMotor.class, "FLDC");
        backLeftMotor = hwMap.get(DcMotor.class, "RLDC");
        frontRightMotor = hwMap.get(DcMotor.class, "FRDC");
        backRightMotor = hwMap.get(DcMotor.class, "RRDC");


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
/*
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

*/
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (isGobildaPinpointIMU) {
            isPinpoint = true;
            // Read the same configuration as Pedro's PinpointLocalizer, including units.
            // Future pod calibration changes belong in Constants.localizerConfig only.
            PinpointConfig config = Constants.localizerConfig;
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, config.name.get());
            pinpoint.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(),
                    config.offsetUnits.get());
            if (config.ticksPerUnit.get().isPresent()) {
                pinpoint.setEncoderResolution(config.ticksPerUnit.get().getAsDouble(),
                        config.encoderResolutionUnit.get());
            } else {
                pinpoint.setEncoderResolution(config.podType.get());
            }
            pinpoint.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());
            // Preserve existing init behavior: this resets pose rather than carrying Auto forward.
            pinpoint.resetPosAndIMU();


            // Set the location of the robot - the place the robot starts
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
        }
        else {
            isPinpoint = false;
            imu = hwMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

            imu.initialize(new IMU.Parameters(RevOrientation));
        }
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 0.3;
        double maxSpeed = 0.3;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        if (isPinpoint) {
            Pose2D pose2D = getPosition();
            theta = AngleUnit.normalizeRadians(theta - pose2D.getHeading(AngleUnit.RADIANS));
        } else {
            theta = AngleUnit.normalizeRadians(theta -
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void updatePinpointPosition(double x, double y, double heading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading));
    }

    public void updatePinpoint() {
        pinpoint.update();
    }


}
