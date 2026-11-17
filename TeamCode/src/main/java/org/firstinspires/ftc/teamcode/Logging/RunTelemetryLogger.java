/* ============================================================================
 * What it does: Saves displayed telemetry at 10 Hz and control/state events to CSV.
 * Applications: BioBuzzTeleop and RED/BLUE Flower, Garden, Garden_NoFlwr Auto.
 * Writes on a bounded background queue; logging failure never stops robot control.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.Logging;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;
import java.util.concurrent.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public final class RunTelemetryLogger {
    private File runFile;
    private long lastSampleCostNanos;
    private final Map<String,String> values = new LinkedHashMap<>();
    private final Map<String,String> events = new HashMap<>();
    private final BlockingQueue<String> queue = new ArrayBlockingQueue<>(128);
    private volatile boolean closing;
    private boolean enabled = true, backWasPressed;
    private volatile String status = "Not opened";
    private volatile long dropped;
    private Thread worker;
    private long origin, lastSample, lastVoltage;
    private String path = "";
    private String opModeName = "Unknown";

    /** Capture eager addData/addLine values while preserving the real telemetry calls. */
    public Telemetry attach(Telemetry delegate, String mode) {
        return attach(delegate, mode, new File("/sdcard/FIRST/logs/runs"));
    }
    // Directory injection supports desktop validation without robot hardware.
    Telemetry attach(Telemetry delegate, String mode, File dir) {
        opModeName = mode;
        origin = System.nanoTime();
        try {
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create " + dir);
            File file = File.createTempFile("BioBuzzRun-" + mode + "-" + System.currentTimeMillis() + "-", ".csv", dir);
            runFile = file;
            synchronized(RunLogRetention.class) { RunLogRetention.ACTIVE.add(file.getAbsolutePath()); }
            path = file.getAbsolutePath();
            BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file), java.nio.charset.StandardCharsets.UTF_8));
            writer.write("host_unix_ms,elapsed_ms,type,field,value\n");
            writer.flush();
            status = "Recording";
            worker = new Thread(() -> writeLoop(writer), "BioBuzz-CSV");
            worker.setDaemon(true);
            worker.start();
            event("lifecycle", "INIT");
        } catch (IOException | RuntimeException e) {
            status = "LOG ERROR: " + e.getMessage();
            synchronized(RunLogRetention.class) { if(runFile!=null)RunLogRetention.ACTIVE.remove(runFile.getAbsolutePath()); }
        }
        return (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class}, (proxy, method, args) -> {
                    // OFF bypasses CSV capture/formatting; normal DS telemetry still runs.
                    if (!enabled) {
                        try { return method.invoke(delegate, args); }
                        catch (InvocationTargetException e) { throw e.getCause(); }
                    }
                    boolean formatted = args != null && args.length == 3
                            && method.getName().equals("addData") && args[2] instanceof Object[];
                    try {
                        if (args != null && args.length >= 2 && method.getName().equals("addData")) {
                            String key = String.valueOf(args[0]);
                            // Validate formatting before forwarding it; malformed values become error text.
                            if (!key.startsWith("CSV ")) {
                                String value = formatted
                                        ? String.format(Locale.US, String.valueOf(args[1]), (Object[]) args[2])
                                        : String.valueOf(args[1]);
                                values.put(key, value);
                            }
                        } else if (args != null && args.length == 1 && method.getName().equals("addLine")) {
                            changed("message", String.valueOf(args[0]));
                        }
                    } catch (IllegalFormatException e) {
                        // Do not forward the malformed formatter to DS for later evaluation.
                        String fallback = "FORMAT ERROR: " + e.getClass().getSimpleName();
                        status = fallback;
                        values.put(String.valueOf(args[0]), fallback);
                        return delegate.addData(String.valueOf(args[0]), (Object) fallback);
                    } catch (RuntimeException e) {
                        // Capture failures must not prevent the original telemetry call.
                        status = "LOG CAPTURE ERROR: " + e.getClass().getSimpleName();
                    }
                    try { return method.invoke(delegate, args); }
                    catch (InvocationTargetException e) { throw e.getCause(); }
                });
    }

    /** Ignore BACK held at START; a fresh press toggles once, release rearms it. */
    public void beginRun(boolean backPressed) { backWasPressed = backPressed; }
    public void updateToggle(boolean backPressed) {
        if (backPressed && !backWasPressed) {
            if (enabled) {
                event("logging", "PAUSED");
                enabled = false;
            } else {
                values.clear();
                events.clear();
                lastSample = lastVoltage = 0;
                enabled = true;
                event("logging", "RESUMED");
            }
        }
        backWasPressed = backPressed;
    }
    public boolean isEnabled() { return enabled; }
    private static boolean isState(String k) {
        return k.endsWith("stage") || k.equals("Shooting sequence") || k.equals("Intake actual output")
                || k.equals("Last waypoint timeout") || k.equals("LL frame state");
    }
    public void event(String field, String value) {
        // All instrumented OpModes share this named lifecycle marker.
        if ("lifecycle".equals(field) && "START".equals(value))
            offer(row("EVENT", field, "START OpMode=" + opModeName));
        else if (enabled) offer(row("EVENT", field, value));
    }
    public void changed(String field, String value) {
        if (!enabled) return;
        if (!Objects.equals(events.put(field, value), value)) event(field, value);
    }
    public void gamepads(Gamepad one, Gamepad two) {
        if (!enabled) return;
        changed("gamepad1 buttons", buttons(one));
        changed("gamepad2 buttons", buttons(two));
        values.put("gamepad1 sticks LX/LY/RX/RY", sticks(one));
        values.put("gamepad2 sticks LX/LY/RX/RY", sticks(two));
    }
    private static String sticks(Gamepad g) {
        return String.format(Locale.US,"%.3f/%.3f/%.3f/%.3f",g.left_stick_x,g.left_stick_y,g.right_stick_x,g.right_stick_y);
    }
    private static String buttons(Gamepad g) {
        return "A="+g.a+" B="+g.b+" X="+g.x+" Y="+g.y+" UP="+g.dpad_up+" DOWN="+g.dpad_down
                +" LEFT="+g.dpad_left+" RIGHT="+g.dpad_right+" LT="+(g.left_trigger>0.5)
                +" RT="+(g.right_trigger>0.5)+" LB="+g.left_bumper+" RB="+g.right_bumper
                +" START="+g.start+" BACK="+g.back;
    }
    /** Call after adding the current loop's telemetry. Battery read is limited to 1 Hz. */
    public void sample(HardwareMap map, Telemetry telemetry) {
        long began = System.nanoTime();
        try { if (enabled) sampleSafely(map, telemetry); }
        catch (RuntimeException e) { status = "LOG SAMPLE ERROR: " + e.getClass().getSimpleName(); }
        lastSampleCostNanos = System.nanoTime()-began;
        telemetry.addData("CSV sample cost ms", "%.3f", lastSampleCostNanos/1e6);
        telemetry.addData("CSV logging (G1 BACK)", enabled ? "ON" : "OFF / PAUSED");
        telemetry.addData("CSV OpMode", opModeName);
        telemetry.addData("CSV file",path);
        telemetry.addData("CSV status",status+"; dropped batches="+dropped);
    }
    private void sampleSafely(HardwareMap map, Telemetry telemetry) {
        long now = System.nanoTime();
        if (now-lastVoltage >= 1_000_000_000L) {
            double v = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : map.voltageSensor) {
                double reading = sensor.getVoltage();
                if (reading > 0) v = Math.min(v, reading);
            }
            values.put("Battery voltage", Double.isFinite(v) ? String.format(Locale.US,"%.3f",v) : "Unavailable");
            lastVoltage = now;
        }
        // Record discrete state transitions immediately, independent of the 10 Hz samples.
        for (Map.Entry<String,String> e : values.entrySet()) {
            String k=e.getKey();
            if (isState(k)) changed(k,e.getValue());
        }
        if (lastSample == 0 || now-lastSample >= 100_000_000L) {
            StringBuilder batch = new StringBuilder();
            for (Map.Entry<String,String> e : values.entrySet()) batch.append(row("SAMPLE",e.getKey(),e.getValue()));
            offer(batch.toString());
            lastSample = now;
        }
    }
    private String row(String type,String key,String value) {
        return System.currentTimeMillis()+","+(System.nanoTime()-origin)/1_000_000+","+type+","+quote(key)+","+quote(value)+"\n";
    }
    private static String quote(String value) { return "\""+value.replace("\"","\"\"")+"\""; }
    private void offer(String text) {
        if (worker != null && !closing && !queue.offer(text)) dropped++;
    }
    private void writeLoop(BufferedWriter writer) {
        try (BufferedWriter out = writer) {
            long flushed=System.nanoTime();
            while (!closing || !queue.isEmpty()) {
                String batch=queue.poll(100,TimeUnit.MILLISECONDS);
                if (batch!=null) {
                    out.flush(); // Include previous buffered bytes in retention accounting.
                    boolean room=RunLogRetention.makeRoom(runFile.getParentFile(),runFile,
                        batch.getBytes(java.nio.charset.StandardCharsets.UTF_8).length,
                        org.firstinspires.ftc.teamcode.config.LoggingConfig.MAX_RUNS,
                        (long)(org.firstinspires.ftc.teamcode.config.LoggingConfig.BUDGET_BYTES
                            * org.firstinspires.ftc.teamcode.config.LoggingConfig.CLEANUP_FRACTION));
                    if(!room || runFile.getParentFile().getUsableSpace()
                            < org.firstinspires.ftc.teamcode.config.LoggingConfig.MIN_FREE_BYTES) {
                        status="LOG STORAGE LIMIT: logging stopped";
                        closing=true;queue.clear();break;
                    }
                    out.write(batch);
                }
                if (System.nanoTime()-flushed>=1_000_000_000L) { out.flush(); flushed=System.nanoTime(); }
            }
            out.flush(); if(!status.startsWith("LOG STORAGE"))status="Saved";
        } catch (IOException | InterruptedException | RuntimeException e) { status="LOG ERROR: "+e.getMessage(); closing=true; }
        finally { synchronized(RunLogRetention.class) { if(runFile!=null)RunLogRetention.ACTIVE.remove(runFile.getAbsolutePath()); } }
    }
    /** Call after stopping motors. Drain asynchronously if storage takes longer than 200 ms. */
    public void close() {
        offer(row("EVENT","lifecycle","STOP")); // Preserve lifecycle marker even while paused.
        closing=true;
        if (worker!=null) try { worker.join(200); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
}
