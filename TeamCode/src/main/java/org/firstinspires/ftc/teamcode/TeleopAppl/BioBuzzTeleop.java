package org.firstinspires.ftc.teamcode.TeleopAppl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/** Driver-controlled chassis mode based on KKN_TEST's active mecanum test. */
@TeleOp(name = "BioBuzzTeleop", group = "BioBuzz")
public class BioBuzzTeleop extends OpMode {
    private static final double TURN_DEADBAND = 0.05;
    // Keep below MecanumDrive's 0.3 normalization floor so pure turning stays slower.
    private static final double MAX_TURN_INPUT = 0.20;
    private MecanumDrive driveTest;
    private Intake intake;
    private boolean intakeRunning, intakeManualStop;
    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    @Override public void init() {
        driveTest = new MecanumDrive();
        // false selects the REV hub IMU named "imu"; true resets/configures Pinpoint.
        // Robot-relative drive() below does not use either sensor's heading.
        driveTest.init(hardwareMap, false);
        driveTest.drive(0, 0, 0);
        intake = new Intake();
        intake.init(hardwareMap); // Same "intake" motor and -1.0 power as KKN_TEST's helper.
        intake.stop();
        intakeRunning = false;
        intakeManualStop = false;
        // MecanumDrive owns motor directions and encoder modes, matching KKN_TEST.
        frontLeft = hardwareMap.get(DcMotor.class, "FLDC");
        frontRight = hardwareMap.get(DcMotor.class, "FRDC");
        rearLeft = hardwareMap.get(DcMotor.class, "RLDC");
        rearRight = hardwareMap.get(DcMotor.class, "RRDC");
        telemetry.addLine("Ready: left stick drive/strafe, right stick X turn");
        telemetry.addLine("Intake: auto-start on drive; D-pad UP start / DOWN stop (latched)");
    }

    @Override public void loop() {
        // Robot-relative controls: left stick Y drives forward/backward (negated because

        //  Left stick up/down: drives forward/backward.
        //  Left stick/right: strafes the robot sideways left/right, keeping its facing direction.
        //  Right stick left/right: rotates the robot anticlockwise/clockwise.
        //  dpad up - Manually start Intake
        //  dpad down - Manually stop Intake
        //  Intake will automatically start once robo moves
        // Use Dpad down to stop Intake


        // Movement follows the robot's current front/right, not fixed field directions;
        // drive() does not use IMU or Pinpoint heading to compensate for robot rotation.
        // Ignore stick drift and square the remaining input for gentle center response.
        // Preserve direction; releasing the stick immediately commands zero turn.
        double stickTurn = gamepad1.right_stick_x;
        double magnitude = Math.max(0, (Math.abs(stickTurn) - TURN_DEADBAND)
                / (1 - TURN_DEADBAND));
        double turn = Math.copySign(MAX_TURN_INPUT * magnitude * magnitude, stickTurn);
        driveTest.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);
        // Start on any driving axis, ignoring small stick drift. Right-stick Y is unused.
        boolean driving = Math.abs(gamepad1.left_stick_y) > TURN_DEADBAND
                || Math.abs(gamepad1.left_stick_x) > TURN_DEADBAND
                || Math.abs(stickTurn) > TURN_DEADBAND;
        // Manual stop takes priority, even while driving or if both buttons are pressed.
        // Keep it latched so joystick input cannot undo the driver's stop command.
        if (gamepad1.dpad_down) {
            intakeManualStop = true;
            intakeRunning = false;
        } else if (gamepad1.dpad_up) {
            intakeManualStop = false;
            intakeRunning = true; // Can also collect while the chassis is stationary.
        } else if (driving && !intakeManualStop) {
            intakeRunning = true;
        }
        // Releasing the sticks does not stop intake; use D-pad DOWN or OpMode STOP.
        if (intakeRunning) intake.start();
        else intake.stop();
        telemetry.addData("Intake", intakeManualStop ? "Manual STOP (UP to restart)"
                : intakeRunning ? "Running" : "Waiting for drive or D-pad UP");
        telemetry.addData("Turn input", "%.2f", turn);
        telemetry.addData("Drive", "Robot-relative");
        telemetry.addData("FL / FR power", "%.2f / %.2f", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("RL / RR power", "%.2f / %.2f", rearLeft.getPower(), rearRight.getPower());
    }

    @Override public void stop() {
        if (intake != null) intake.stop();
        if (driveTest != null) driveTest.drive(0, 0, 0);
        // KKN_TEST's inactive color sensor is not initialized or dereferenced here.
    }
}
