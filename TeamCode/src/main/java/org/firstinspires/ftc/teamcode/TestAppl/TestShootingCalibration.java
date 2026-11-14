/* ============================================================================
 * Class: TestShootingCalibration
 * What it does: Independently tests pollen velocity, feeder power and intake power.
 * Applications using it: TestShootingCalibration (Driver Station test TeleOp).
 * Uses TeleopConfig defaults; no arm, helper, driving, nectar or vision hardware.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TestAppl;

import static org.firstinspires.ftc.teamcode.config.TeleopConfig.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.InputDCShooter;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooter;

@com.qualcomm.robotcore.eventloop.opmode.Disabled
@TeleOp(name = "TestShootingCalibration", group = "Test")
public class TestShootingCalibration extends OpMode {
    private static final int FEEDER_STEP_PERCENT = 1;
    private final Intake intake = new Intake();
    private final InputDCShooter feeder = new InputDCShooter();
    private PollenShooter shooter;
    // Read-only references to motors already initialized by the subsystems.
    private DcMotor intakeMotor, feederMotor, pollenMotor;
    private int intakePercent, shooterPercent, feederPercent;
    private boolean intakeRunning, shooterRunning, feederRunning;
    private boolean[] previous = new boolean[9];

    // Six adjustment buttons followed by three start buttons.
    private boolean[] buttons() {
        return new boolean[]{gamepad1.y, gamepad1.x, gamepad2.dpad_right,
            gamepad2.dpad_left, gamepad2.dpad_up, gamepad2.dpad_down,
            gamepad1.dpad_right, gamepad2.left_trigger > 0.5, gamepad2.a};
    }
    @Override public void init() {
        intake.init(hardwareMap, false);
        feeder.init(hardwareMap);
        shooter = PollenShooter.getInstance();
        shooter.initialize(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        feederMotor = hardwareMap.get(DcMotor.class, InputDCShooter.HARDWARE_NAME);
        pollenMotor = hardwareMap.get(DcMotor.class, "shooter");
        intakePercent = INITIAL_INTAKE_POWER_PERCENT;
        shooterPercent = INITIAL_SHOOTER_SPEED_PERCENT;
        feederPercent = (int) Math.round(Math.abs(FEEDER_POWER) * 100);
        stop();
        previous = buttons();
        showTelemetry();
    }
    @Override public void init_loop() { previous = buttons(); showTelemetry(); }
    @Override public void start() { previous = buttons(); }

    @Override public void loop() {
        boolean[] held = buttons();
        boolean[] pressed = new boolean[held.length];
        for (int i = 0; i < held.length; i++) pressed[i] = held[i] && !previous[i];
        if (pressed[0] && !held[1]) intakePercent = Math.min(MAX_INTAKE_POWER_PERCENT, intakePercent + INTAKE_STEP_PERCENT);
        if (pressed[1] && !held[0]) intakePercent = Math.max(0, intakePercent - INTAKE_STEP_PERCENT);
        if (pressed[2] && !held[3]) shooterPercent = Math.min(MAX_SHOOTER_SPEED_PERCENT, shooterPercent + SHOOTER_STEP_PERCENT);
        if (pressed[3] && !held[2]) shooterPercent = Math.max(0, shooterPercent - SHOOTER_STEP_PERCENT);
        if (pressed[4] && !held[5]) feederPercent = Math.min(100, feederPercent + FEEDER_STEP_PERCENT);
        if (pressed[5] && !held[4]) feederPercent = Math.max(0, feederPercent - FEEDER_STEP_PERCENT);
        // Stop takes priority. Motors are independent: there is no feeding interlock.
        if (gamepad1.dpad_left) intakeRunning = false;
        else if (pressed[6]) intakeRunning = true;
        if (gamepad2.right_trigger > 0.5) shooterRunning = false;
        else if (pressed[7]) shooterRunning = true;
        if (gamepad2.b) feederRunning = false;
        else if (pressed[8]) feederRunning = true;
        // Zero stops the motor; increasing from zero requires a fresh start press.
        if (intakePercent == 0) intakeRunning = false;
        if (shooterPercent == 0) shooterRunning = false;
        if (feederPercent == 0) feederRunning = false;
        if (intakeRunning) intake.start(intakePercent / 100.0, false, false);
        else intake.stop();
        if (shooterRunning) shooter.runSpeedFraction(shooterPercent / 100.0);
        else shooter.stopImmediately();
        // Reverse the TeleOp feeder direction for this calibration test.
        feeder.setPower(-Math.copySign(feederPercent / 100.0, FEEDER_POWER));
        if (feederRunning) feeder.start();
        else feeder.stop();
        previous = held;
        showTelemetry();
    }
    private void showTelemetry() {
        telemetry.addLine("Independent motors; controls active after DS START");
        telemetry.addLine("GP1 intake: RIGHT start / LEFT stop; Y increase / X decrease");
        telemetry.addLine("GP2 pollen: LT start / RT stop; RIGHT increase / LEFT decrease");
        telemetry.addLine("GP2 feeder: A start / B stop; UP increase / DOWN decrease");
        telemetry.addData("Intake", "%s | selected %d%% power", intakeRunning ? "Running" : "Stopped", intakePercent);
        telemetry.addData("Feeder", "%s | selected %d%% power", feederRunning ? "Running" : "Stopped", feederPercent);
        // Match BioBuzzTeleop labels; show percentage readings before encoder details.
        telemetry.addData("Pollen shooter", shooterRunning ? "Running (velocity control)" : "Stopped (coasts)");
        telemetry.addData("Pollen selected speed (% of reference)", "%d%%", shooterPercent);
        telemetry.addData("Pollen SDK power (not measured RPM)", "%.0f%%", shooter.getShooterPower() * 100);
        double target = shooter.getTargetVelocity(), measured = shooter.measuredVelocity();
        telemetry.addData("Pollen target / measured (ticks/s)", "%.0f / %.0f", target, measured);
        telemetry.addData("Pollen error (ticks/s)", "%.0f", target - measured);
        showMotorTelemetry("Intake", intakeMotor);
        showMotorTelemetry("Feeder", feederMotor);
        // Pollen power and velocity are already displayed above.
        telemetry.addLine("Encoder speed requires connected encoder wiring; zero alone does not prove stopped.");
    }
    /** SDK power is a command, not measured shaft speed or electrical power. */
    private void showMotorTelemetry(String name, DcMotor motor) {
        telemetry.addData(name + " SDK commanded power", "%.1f%%", motor.getPower() * 100);
        telemetry.addData(name + " mode / direction", "%s / %s", motor.getMode(), motor.getDirection());
        telemetry.addData(name + " encoder position (ticks)", motor.getCurrentPosition());
        if (motor instanceof DcMotorEx) {
            telemetry.addData(name + " encoder velocity (ticks/s)", "%.0f", ((DcMotorEx) motor).getVelocity());
        } else {
            telemetry.addData(name + " encoder velocity", "Unavailable for this motor interface");
        }
    }
    @Override public void stop() {
        intakeRunning = shooterRunning = feederRunning = false;
        intake.stop(); feeder.stop();
        if (shooter != null) shooter.stopImmediately();
    }
}