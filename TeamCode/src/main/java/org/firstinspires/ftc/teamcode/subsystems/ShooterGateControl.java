/* ============================================================================
 * What it does: Pure startup/recovery timing for a continuously running feeder
 *   and speed-protected positional gate. No hardware calls or blocking delays.
 * Applications: BioBuzzTeleop, AutoShootSequence.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.config.TeleopConfig.*;

public final class ShooterGateControl {
    public enum State { IDLE, SPINNING_UP, READY, RECOVERING }
    private State state = State.IDLE;
    private boolean feederRunning, gateOpen;
    private double startedAt, feederStartedAt, lowSince = Double.NaN,
            readySince = Double.NaN, allowedSince = Double.NaN;
    public void start(double now) { stop(); startedAt = now; state = State.SPINNING_UP; }
    public void stop() {
        state = State.IDLE; feederRunning = gateOpen = false;
        lowSince = readySince = allowedSince = Double.NaN;
    }
    /** Close immediately and restart the opening dwell, preserving rotating motors. */
    public void closeGate() { gateOpen = false; allowedSince = Double.NaN; }
    /** Direction changes require measured recovery, with no timed bypass. */
    public void requireRecovery() {
        if (state == State.IDLE) return;
        state = State.RECOVERING;
        lowSince = readySince = Double.NaN;
        closeGate();
    }
    public void update(double now, double target, double measured, boolean requested, double openDelay) {
        if (state == State.IDLE) return;
        if (!Double.isFinite(target) || target <= 0) { stop(); return; }
        boolean finite = Double.isFinite(measured);
        boolean ready = finite && Math.abs(target-measured)
                <= Math.min(SHOOTER_READY_MAX_ERROR_TICKS, target*SHOOTER_READY_ERROR_FRACTION);
        if (state == State.SPINNING_UP) {
            if (ready || now-startedAt >= SHOOTER_START_TIMEOUT_SECONDS) {
                feederRunning = true; feederStartedAt = now; state = State.READY;
            }
        } else if (state == State.READY && now-feederStartedAt >= openDelay) {
            boolean slow = !finite || measured < target*(1-SHOOTER_DROP_FRACTION);
            if (!slow) lowSince = Double.NaN;
            else {
                if (Double.isNaN(lowSince)) lowSince = now;
                if (now-lowSince >= SHOOTER_DROP_HOLD_SECONDS) requireRecovery();
            }
        } else if (state == State.RECOVERING) {
            if (!ready) readySince = Double.NaN;
            else {
                if (Double.isNaN(readySince)) readySince = now;
                if (now-readySince >= SHOOTER_RECOVERY_HOLD_SECONDS) {
                    state = State.READY; lowSince = Double.NaN;
                    if (!feederRunning) { feederRunning = true; feederStartedAt = now; }
                }
            }
        }
        // Loss of pose/request/recovery closes now; cannot reopen from an old request timer.
        if (!requested || state != State.READY || !feederRunning) closeGate();
        else {
            if (Double.isNaN(allowedSince)) allowedSince = now;
            gateOpen = now-allowedSince >= openDelay && now-feederStartedAt >= openDelay;
        }
    }
    public boolean isFeederRunning() { return feederRunning; }
    public boolean isGateOpen() { return gateOpen; }
    public State getState() { return state; }
}
