/* ============================================================================
 * Class: IntakeServoCalibration
 * What it does: Manually steps the intake servo command to calibrate arm positions.
 * Applications using it: IntakeServoCalibration (standalone Driver Station test).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TestAppl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeArmSingleServo;

/** Command display only: a three-wire servo does not report physical arm angle. */
@TeleOp(name = "IntakeServoCalibration", group = "Test")
public class IntakeServoCalibration extends OpMode {
    private static final double RANGE_DEGREES = 300.0;
    private static final double STEP_DEGREES = 5.0;
    private ServoImplEx servo;
    private double selectedDegrees = 170.0;
    private boolean enabled, aWasPressed, upWasPressed, downWasPressed;

    @Override public void init() {
        // Do not call arm.init(): that would command its UP preset into the linkage.
        servo = hardwareMap.get(ServoImplEx.class, IntakeArmSingleServo.HARDWARE_NAME);
        servo.setPwmDisable();
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        selectedDegrees = 150.0;
        enabled = false;
        showStatus();
    }

    @Override public void init_loop() { showStatus(); }

    @Override public void start() {
        aWasPressed = gamepad1.a;
        upWasPressed = gamepad1.dpad_up;
        downWasPressed = gamepad1.dpad_down;
    }

    @Override public void loop() {
        // B wins over all other inputs. Disabled PWM releases holding torque.
        if (gamepad1.b) {
            servo.setPwmDisable();
            enabled = false;
        } else if (gamepad1.a && !aWasPressed && !gamepad1.start) {
            servo.setPosition(selectedDegrees / RANGE_DEGREES);
            servo.setPwmEnable();
            enabled = true;
        } else if (enabled) {
            double next = selectedDegrees;
            // Numeric direction only; physical UP/DOWN depends on horn mounting.
            if (gamepad1.dpad_up && !upWasPressed && !gamepad1.dpad_down)
                next = Math.min(RANGE_DEGREES, selectedDegrees + STEP_DEGREES);
            else if (gamepad1.dpad_down && !downWasPressed && !gamepad1.dpad_up)
                next = Math.max(0, selectedDegrees - STEP_DEGREES);
            if (next != selectedDegrees) {
                selectedDegrees = next;
                servo.setPosition(selectedDegrees / RANGE_DEGREES);
            }
        }
        aWasPressed = gamepad1.a;
        upWasPressed = gamepad1.dpad_up;
        downWasPressed = gamepad1.dpad_down;
        showStatus();
    }

    private void showStatus() {
        telemetry.addLine("Disconnect linkage for initial calibration. No physical angle feedback.");
        telemetry.addLine("After START: Gamepad 1 A enable; B disable PWM (arm may fall).");
        telemetry.addLine("D-pad UP +5 degrees / DOWN -5 degrees, once per press while enabled.");
        telemetry.addData("Selected command angle (deg)", selectedDegrees);
        telemetry.addData("Normalized command", "%.3f", selectedDegrees / RANGE_DEGREES);
        telemetry.addData("Output", enabled ? "Enabled" : "Disabled - selected angle is not being driven");
        telemetry.addData("PWM enabled", servo.isPwmEnabled());
        telemetry.addData("Port", servo.getConnectionInfo());
        telemetry.addLine("Record angles manually; this test does not change UP/DOWN presets.");
    }

    @Override public void stop() {
        if (servo != null) servo.setPwmDisable();
        enabled = false;
    }
}