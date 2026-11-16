/* ============================================================================
 * What it does: Spins Pollen/intake from START, starts feeder at speed or 1.5 s,
 *   and releases balls only through a position-authorized, speed-protected gate.
 * Applications: RED/BLUE AutoFlower, AutoGarden, AUTO_Garden_NoFlwr.
 * Hive decisions belong to those applications; their existing checks are retained.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.function.BooleanSupplier;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;

public class AutoShootSequence {
    private final Intake intake = new Intake();
    private final InputDCShooter feeder = new InputDCShooter();
    private final PollenShooter shooter = PollenShooter.getInstance();
    private final ShooterGateServo gate = new ShooterGateServo();
    private final ShooterGateControl control = new ShooterGateControl();
    private boolean initialized, running, shotActive;
    private BooleanSupplier atPosition = () -> false;
    private double shotOpenedAt = Double.NaN, shotDuration;
    private String status = "Not initialized";

    public void init(HardwareMap map) {
        gate.init(map); // Positional CLOSED immediately; no CRServo lookup.
        intake.init(map);
        shooter.initialize(map);
        feeder.init(map);
        feeder.setPower(-FEEDER_POWER); // Same tested direction as TeleOp.
        initialized = true;
        stop();
    }
    /** START: Pollen first, intake immediately after; feeder readiness runs during travel too. */
    public void startAuto() {
        if (!initialized) throw new IllegalStateException("Initialize shooting sequence first");
        stop();
        shooter.runSpeedFraction(SHOOTER_SPEED_FRACTION);
        intake.start(INTAKE_POWER);
        control.start(System.nanoTime()/1e9);
        running = true;
        status = "Pollen + intake running; gate CLOSED; feeder waiting";
    }
    /** Called every Auto loop, including movement, hive waits and parking. */
    public void update() {
        if (!running) return;
        double now = System.nanoTime()/1e9;
        // Duration starts at the first OPEN command; recovery pauses do not extend it.
        if (shotActive && !Double.isNaN(shotOpenedAt) && now-shotOpenedAt >= shotDuration)
            closeGateKeepMotors();
        boolean permitted = shotActive && atPosition.getAsBoolean();
        control.update(now, shooter.getTargetVelocity(), shooter.measuredVelocity(), permitted,
                GATE_OPEN_DELAY_SECONDS);
        if (control.isFeederRunning()) feeder.start(); else feeder.stop();
        if (control.isGateOpen()) {
            gate.open();
            if (Double.isNaN(shotOpenedAt)) shotOpenedAt = now;
        } else gate.close();
        status = control.getState()+"; gate "+(gate.isOpen()?"OPEN":"CLOSED")
                +"; "+(shotActive?(permitted?"shot authorized":"waiting for shooting pose"):"travel / waiting");
    }
    /** Close before leaving a shot, on parking entry, or if a command is cancelled. */
    public void closeGateKeepMotors() {
        shotActive = false;
        atPosition = () -> false;
        control.closeGate();
        gate.close();
        status = "Gate CLOSED; motors keep running until parking ends";
    }
    public Command positionShotCommand(double seconds, BooleanSupplier positionCheck) {
        if (!Double.isFinite(seconds) || seconds <= 0 || positionCheck == null)
            throw new IllegalArgumentException("Invalid shooting command");
        return new LambdaCommand("Position-authorized gate shot")
                .setStart(() -> {
                    closeGateKeepMotors();
                    shotDuration = seconds;
                    shotOpenedAt = Double.NaN;
                    atPosition = positionCheck;
                    shotActive = running;
                })
                .setUpdate(this::update)
                .setIsDone(() -> !shotActive)
                .setStop(interrupted -> closeGateKeepMotors())
                .requires(this);
    }
    /** Terminal stop: never leave an opening request that can restart after STOP/deadline. */
    public void stop() {
        closeGateKeepMotors();
        running = false;
        control.stop();
        feeder.stop();
        if (initialized) intake.stop();
        shooter.stopImmediately();
        status = "Stopped; gate CLOSED";
    }
    public String getStatus() { return status; }
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter gate", gate.isOpen()?"OPEN":"CLOSED");
        telemetry.addData("Shooter gate commanded degrees",gate.getCommandedDegrees());
        telemetry.addData("Pollen target (ticks/s)",shooter.getTargetVelocity());
        telemetry.addData("Pollen measured (ticks/s)",shooter.measuredVelocity());
        telemetry.addData("Feeder running",control.isFeederRunning());
    }
}
