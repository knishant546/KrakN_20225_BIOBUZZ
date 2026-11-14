/* ============================================================================
 * Class: IntakeArmOnlyTest
 * What it does: Tests only the intake-arm servo using the shared UP/DOWN presets.
 * Applications using it: Intake Arm Only Test (Driver Station test OpMode).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TestAppl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeArmSingleServo;

/** Same hardware name, PWM range, direction and angles as BioBuzzTeleop.
 * No drive, intake DC, shooter, feeder, camera or odometry initialization.
 */
@Disabled
@TeleOp(name = "Intake Arm Only Test", group = "Test")
public class IntakeArmOnlyTest extends OpMode {
    private final IntakeArmSingleServo arm = new IntakeArmSingleServo();
    private String position = "Waiting for START";

    @Override public void init() {
        // Shared init() moves immediately, so defer it until START.
        telemetry.setMsTransmissionInterval(100);
        showInstructions();
    }
    @Override public void init_loop() { showInstructions(); }
    @Override public void start() {
        arm.init(hardwareMap); // Commands UP using the shared servo settings.
        position = "UP";
    }
    @Override public void loop() {
        // UP has priority; otherwise retain the last commanded position.
        if (gamepad1.dpad_up) {
            arm.liftUp();
            position = "UP";
        } else if (gamepad1.dpad_down) {
            arm.liftDown();
            position = "DOWN";
        }
        showInstructions();
        telemetry.addData("Commanded angle (deg)", arm.getCommandedDegrees());
        telemetry.addLine("Commanded angle only, not measured physical position.");
    }
    private void showInstructions() {
        telemetry.addLine("Servo only: START moves arm UP.");
        telemetry.addLine("Gamepad 1 D-pad UP = fold up; DOWN = lower.");
        telemetry.addData("Hardware name", IntakeArmSingleServo.HARDWARE_NAME);
        telemetry.addData("UP / DOWN presets (deg)", "%.1f / %.1f",
                IntakeArmSingleServo.UP_DEGREES, IntakeArmSingleServo.DOWN_DEGREES);
        telemetry.addData("Position", position);
    }
    @Override public void stop() {
        // No extra movement on STOP; SDK handles OpMode shutdown.
    }
}