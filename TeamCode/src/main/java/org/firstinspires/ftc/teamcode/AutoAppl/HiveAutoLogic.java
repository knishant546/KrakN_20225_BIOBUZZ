package org.firstinspires.ftc.teamcode.AutoAppl;

/** Hardware-independent single-shot interlock: never queue an unchecked second shot. */
public final class HiveAutoLogic {
    public enum Phase { CHECK, FEED, SETTLE, DONE, FAULT }
    public enum Hive { UP, DOWN, UNKNOWN }
    public Phase phase = Phase.DONE;
    // Estimated remaining load and attempted feeds; neither is a confirmed exit count.
    public int balls, attempts;
    public boolean feed;
    public String reason = "Idle";
    private double began, batchAt, phaseAt, lastFrame, afterShotFrame;
    private int stable;
    public void begin(double now, int inventory) {
        balls=inventory; attempts=0; feed=false; stable=0;
        began=batchAt=phaseAt=now; lastFrame=afterShotFrame=Double.NaN;
        phase=inventory>0?Phase.CHECK:Phase.DONE; reason="Acquire fresh hive pose";
    }
    private void finish(String why) { feed=false; phase=Phase.DONE; reason=why; }
    public void abort(String why) { feed=false; phase=Phase.FAULT; reason=why; }
    public void tick(double now, Hive hive, double frame, boolean ready) {
        if(phase==Phase.DONE || phase==Phase.FAULT)return;
        // Absolute batch deadline is never reset by feeder cycles or reacquisition.
        if(now-batchAt>=HiveAutoConfig.SHOOTING_TIMEOUT) {
            finish("Shooting timeout; stop shooter and continue route"); return;
        }
        boolean fresh=Double.isFinite(frame) && Double.compare(frame,lastFrame)!=0;
        if(fresh)lastFrame=frame;
        if(phase==Phase.FEED) {
            if(hive!=Hive.UP) { abort("Hive changed during feed; park"); return; }
            if(now-phaseAt>=HiveAutoConfig.FEED_TIME) {
                feed=false; attempts++; balls=Math.max(0,balls-1);
                phase=Phase.SETTLE; phaseAt=now; afterShotFrame=lastFrame; stable=0;
            }
            return;
        }
        if(phase==Phase.SETTLE) {
            // Discard ALL frames captured during ball flight and hive settling.
            if(fresh)afterShotFrame=frame;
            if(now-phaseAt>=HiveAutoConfig.SETTLE) {
                phase=Phase.CHECK; began=phaseAt=now; stable=0;
            }
            return;
        }
        if(hive==Hive.DOWN) { finish("Cell down; skip remaining shots"); return; }
        if(hive!=Hive.UP)stable=0;
        else if(fresh && Double.compare(frame,afterShotFrame)!=0)stable++;
        if(stable>=HiveAutoConfig.STABLE_FRAMES) {
            // No exit sensor or shot-count cutoff: time and hive state end the batch.
            if(ready && now-began>=HiveAutoConfig.SPINUP) {
                phase=Phase.FEED; phaseAt=now; feed=true; stable=0;
                reason="Single shot, then fresh pose check"; return;
            }
        }
        if(now-began>=HiveAutoConfig.ACQUIRE_TIMEOUT)finish("Pose/velocity timeout; skip batch");
    }
}
