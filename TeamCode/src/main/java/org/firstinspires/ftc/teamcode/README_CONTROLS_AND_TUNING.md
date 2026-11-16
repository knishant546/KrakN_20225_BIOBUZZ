# Current positional shooter gate controls — November 16, 2026

This section supersedes the historical controls/tuning snapshot below.

## 1. Controls and sequence

INIT closes the gate and leaves scoring motors stopped. Driver Station START spins Pollen; the feeder starts when velocity error is within min(200 ticks/s, 5% of target), or after 1.5 seconds. Nectar is disabled.

| Controller | Button | Action |
|## Logging

`Logging/RunTelemetryLogger.java` is shared by BioBuzzTeleop and the six scoring Auto modes (RED/BLUE Flower, Garden, Garden_NoFlwr). The Logging folder is beside config, subsystems, AutoAppl and TeleopAppl.

CSV files are stored on the Control Hub at `/sdcard/FIRST/logs/runs/`. Logging defaults ON; GP1 BACK toggles capture without disabling Driver Station telemetry. Snapshots are sampled at 10 Hz; button/state events are also recorded. A bounded background queue performs file writes; overflow drops records rather than waiting in the control loop. CSV status, dropped records and logging timing are displayed in telemetry. These files are data logs, not video recordings.

Download over an existing ADB connection with `adb pull /sdcard/FIRST/logs/runs ./robot-logs`. File timestamps depend on the hub clock; elapsed time is also recorded.

## Current Auto route summary

- Flower: move to first shooting position, first hive check (up to 7 seconds, 3 seconds stability), first shot (5 seconds gate-open timer). Collect only if at least 15 seconds remain. Travel straight/strafe, lower arm, pause 1 second, timed backward collection leg (1 second), collect 2 seconds, retrace to the first shooting position and fold arm at the strafe waypoint. Second shot lasts 6 seconds, then park.
- Full Garden: first shot 3 seconds, Garden collection and return, second hive check (up to 2 seconds, 1 second stability), second shot 3 seconds only if confirmed. Optional Flower collection requires the completed second shot and more than 14 seconds remaining. Return, fold arm, third shot 3 seconds, then clearance and park.
- Garden_NoFlwr: first and second shots 5 seconds each; no Flower trip. Second hive wait allows shooting on timeout. Then park.
- Parking-only: travel to park without starting scoring motors.

All shot durations start at the first gate OPEN command and include later recovery closures. The 24-second parking deadline and 30-second stop override unfinished shots. Timed slide-in legs can finish short of their target; shooting still requires measured position/heading tolerance.

See [config/README.md](config/README.md) for the current constants and waypoint snapshot. Arm presets are UP=20, intermediate=135, DOWN=140 degrees. Auto MAX_PATH_SPEED is currently 1.00; TeleOp drive cap is 0.50. Configure and calibrate the gate separately from the intake arm.
### Independent motor test toggles

GP2 A toggles only Pollen; GP2 B toggles only the feeder, once per press. A/B preserve the LT/RT gate request. Existing readiness protection may close the gate while a motor is stopped; it can reopen when readiness returns without another LT press, unless RT cancelled the request. B overrides automatic feeder startup until re-INIT/START; stopping Pollen with A preserves the feeder's current state. Thus the feeder can be tested with Pollen off and the gate closed. Speed adjustments do not restart a shooter manually stopped with A. START+A/B assignment chords are ignored. Driver Station STOP still stops all motors. Auto is unchanged.
