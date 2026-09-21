package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Pedro Mecanum Test")
public class PedroMecanumTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Use EXACTLY the same Mecanum configuration as Pedro autonomous
        Mecanum drivetrain =
                new Mecanum(hardwareMap, Constants.drivetrainConfig);

        // Very low power for initial testing
        final double TEST_POWER = 0.15;

        telemetry.addLine("Pedro Mecanum Test");
        telemetry.addLine("LEFT STICK Y = Forward/Backward");
        telemetry.addLine("LEFT STICK X = Strafe");
        telemetry.addLine("RIGHT STICK X = Turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*
             * FTC stick Y is negative when pushed forward,
             * therefore use -left_stick_y.
             */
            double forward = -gamepad1.left_stick_y * TEST_POWER;
            double strafe  =  -gamepad1.left_stick_x * TEST_POWER;
            double turn    =  gamepad1.right_stick_x * TEST_POWER;



            drivetrain.drive(
                    new DrivePowers(
                            forward,
                            strafe,
                            turn
                    ),
                    true
            );

            telemetry.addData("Forward command", "%.2f", forward);
            telemetry.addData("Strafe command", "%.2f", strafe);
            telemetry.addData("Turn command", "%.2f", turn);

            telemetry.update();
        }

        drivetrain.stop();
    }
}