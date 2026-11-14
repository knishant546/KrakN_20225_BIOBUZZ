# BioBuzz controls and tuning

Verified against executable code on November 1, 2026. Covers BioBuzzTeleop and RED/BLUE_AutoGarden / RED/BLUE_AutoFlower. Paths below are relative to this README's teamcode directory. Older routines and mechanisms are under AutoAppl/test and subsystems/test; they are not the settings for these main programs. Values are a snapshot: update this document when tuning code.

## 1. Driver Station and controller buttons

Select the desired OpMode, press Driver Station INIT, then Driver Station START. The controller START button is separate from Driver Station START. During TeleOp INIT, shooter and feeder initialize stopped, and the intake arm moves to UP (30 degrees). No hardware-enable button is required.

**Gamepad 1 — driving, intake, and arm**

| Control | Current action |
|---|---|
| Left stick up/down | Drive forward/backward relative to the robot. |
| Left stick left/right | Strafe left/right relative to the robot. |
| Right stick left/right | Turn, with squared input for smooth response; deadband 0.05, maximum turn input 0.20. |
| D-pad RIGHT | Restore intake power to 90% and start intake; clears the latched manual stop. |
| D-pad LEFT | Stop intake and keep it stopped even when driving; RIGHT re-enables it. |
| D-pad UP | Raise intake arm to 30 degrees. |
| D-pad DOWN | Lower intake arm to 150 degrees. |
| Y | Increase intake power by 10 percentage points per press, maximum 100%. Does not move the arm. |
| X | Reduce intake power by 10 percentage points per press, minimum 0%. Initial setting is 90%. |

Driving with left-stick X/Y or right-stick X above 0.05 automatically starts intake unless manually stopped. 
Releasing the sticks does not stop intake. Right-stick Y is unused. 
X decreases and Y increases selected intake power; holding either changes once per press. Both together are ignored. Neither starts a manually stopped intake; 
re-INIT restores the initial intake power. 
RIGHT always restores intake power to 90%, even after X has reduced it to zero. RIGHT takes priority over X/Y adjustment; LEFT still takes priority for stopping. 
Arm priority when multiple controls are held: UP, then DOWN. Gamepad 1 presets take priority over Gamepad 2 arm adjustments. Holding Gamepad 2 UP/DOWN changes once; simultaneous UP+DOWN is ignored. The intermediate servo preset remains available in the subsystem but has no TeleOp button.

**Gamepad 2 — shooter, feeder, and intake reversal**

| Control | Current action                                                                                                                        |
|---|---------------------------------------------------------------------------------------------------------------------------------------|
| LT (>50%) | Start shooter at selected velocity; after 1 second, start feeder (InputDCShooter). Requires a fresh press after Driver Station START. |
| RT (>50%) | Stop feeder immediately; stop shooter 1 second later. Overrides START.                                                                |
| A | Increase shooter selection by 5 percentage points per press, maximum 85%.                                                             |
| B | Decrease shooter selection by 5 percentage points per press, minimum 0%.                                                              |
| X | Toggle intake DC motor direction. Does not start an intake that is stopped.                                                           |
| D-pad UP | It raises from DOWN( floor position). Raise arm by subtracting 5 degrees per press (150 -> 145), minimum 30 degrees.                  |
| D-pad DOWN | It lowers from the TOP UP postion. Lower arm by adding 5 degrees per press (30 -> 35), maximum 150 degrees.                           |

Shooter selection starts at 60% of the configured 2600 ticks/second reference (1560 ticks/second). 
A/B change a running target or the next start's target; they do not start a stopped shooter. 
Selecting zero stops shooter and feeder immediately. 
Holding A/B/X changes once per press. 
START+A/B pairing chords and simultaneous 
A+B do not adjust speed. 
START during an active shooting/stopping sequence does not restart it.

The feeder starts after a fixed 1-second delay, not after measured velocity reaches its target. 
There is no automatic feeder pause based on shooter speed dropping after a ball. 
BACK stops the shooting pair, not the separate intake motor; 
use Gamepad 1 LEFT for intake. 
Driver Station STOP commands drivetrain, intake, feeder, and shooter to stop; shooter FLOAT allows coasting. 
Telemetry displays shooter target/measured/error in ticks/second, selected percentage, SDK power, sequence state, 
intake setting/direction, arm target, and wheel powers.

**Autonomous controls**

Select RED_AutoGarden, RED_AutoFlower, BLUE_AutoGarden, or BLUE_AutoFlower. INIT prepares hardware and paths; Driver Station START runs the sequence automatically without a D-pad command. Driver Station STOP stops the routine. Starting pose is (0, 0, 0). BLUE requires the corresponding physical starting placement and orientation rotated 180 degrees from RED; the same local path coordinates are used.

## 2. Classes and settings for power, direction, and angles

Central tuning files are `config/AutoConfig.java` (all four main Auto programs) and `config/TeleopConfig.java` (BioBuzzTeleop). Edit these files, then rebuild and install the APK. Auto and TeleOp feeder powers are independent. Hardware directions, names, encoder reference, and servo calibration stay in their subsystem classes. Older test applications retain their own route/mechanism settings, but shared drivetrain configuration still applies. Names are more reliable than line numbers after edits. The executable assignments below take precedence over stale comments/telemetry text: current drive values are 0.25, Pedro path speed is 0.80, and arm positions are 30/135/150 degrees.

| Mechanism / setting | File and exact place to edit | Current setting and meaning |
|---|---|---|
| TeleOp wheel command cap | config/TeleopConfig.java — DRIVE_MAX_POWER | 0.25, bounding each wheel's setPower command to +/-0.25; not measured travel speed. |
| TeleOp normalization | config/TeleopConfig.java — DRIVE_NORMALIZATION_FLOOR | 0.25. This is the normalization floor, not a second independent speed cap; joystick response depends on both values. |
| TeleOp wheel direction | MecanumDrive.init(): four setDirection calls | FLDC/RLDC FORWARD; FRDC/RRDC REVERSE. Motors use RUN_USING_ENCODER. |
| TeleOp turning | config/TeleopConfig.java — TURN_DEADBAND, TURN_MAX_POWER (response calculation remains in BioBuzzTeleop.loop()) | 0.05 deadband, 0.10 maximum wheel power when turning alone, squared response. Wheel mixing/cap still applies. |
| Auto drive limit | config/AutoConfig.java — MAX_PATH_SPEED (read by Pedro Constants) | 0.80 Pedro path-speed setting; not a fixed wheel power or inches/second. Auto does not use MecanumDrive.drive(). |
| Auto wheel direction/names | Constants.java — drivetrainConfig: frontLeft/backLeft/frontRight/backRight Name and Direction | FLDC/RLDC FORWARD; FRDC/RRDC REVERSE. Separate settings from TeleOp; keep both configurations consistent with wiring. |
| TeleOp intake initial power | config/TeleopConfig.java — INITIAL_INTAKE_POWER_PERCENT, MAX_INTAKE_POWER_PERCENT, INTAKE_STEP_PERCENT | 90, reset during INIT and whenever D-pad RIGHT starts intake. Gamepad 1 X subtracts 10 and Y adds 10 in loop(), bounded to 0..100%. |
| Auto intake power | config/AutoConfig.java — INTAKE_POWER | 1.00 (100%), used during shooting and collection by all four main Auto routines. |
| Intake direction | subsystems/Intake.java — start(double powerFraction, boolean reverse) | Normal sends negative power; reverse sends positive power. init() maps "intake" but does not explicitly set motor direction or run mode. |
| Feeder power | config/AutoConfig.java and config/TeleopConfig.java — FEEDER_POWER | -0.50 in each config, independently selected during initialization. setPower(...) stores a value for the next start(); it does not immediately update an already-running motor. |
| Feeder direction/mode | InputDCShooter.init() | FORWARD direction, RUN_WITHOUT_ENCODER, BRAKE. Physical rotation combines direction with signed power. |
| Shooter initial/max selection | config/TeleopConfig.java — INITIAL_SHOOTER_SPEED_PERCENT / MAX_SHOOTER_SPEED_PERCENT / SHOOTER_STEP_PERCENT | 60 / 85. SHOOTER_STEP_PERCENT = 5 controls A/B increments. This 85% cap belongs to TeleOp. |
| Auto shooter selection | config/AutoConfig.java — SHOOTER_SPEED_FRACTION | 0.60. Each current Auto passes this into shooting.command(...). Auto does not read TeleOp's selected percentage or its 85% cap. |
| Shooter velocity reference | subsystems/PollenShooter.java — MAX_VELOCITY_TICKS_PER_SECOND | 2600 ticks/second; not RPM and not a measured physical maximum. 60%=1560; 85%=2210; 5 percentage points=130 ticks/second. |
| Shooter direction | PollenShooter.initialize() — shooterMotor.setDirection(...) | REVERSE, with positive velocity targets. |
| Shooter feedback/mode | PollenShooter.initialize(), runVelocity() | RUN_USING_ENCODER and setVelocity(), using REV Hub velocity PIDF/configured motor type. No custom kP/kI/kD values are assigned here. FLOAT on stop. |
| Arm up/intermediate/down | subsystems/IntakeArmSingleServo.java — UP_DEGREES, INTERMEDIATE_DEGREE, DOWN_DEGREES | 30 / 135 / 150 degrees; shared by Auto and TeleOp. INIT commands UP. Flower Auto calls liftDown(), so it uses 150, not INTERMEDIATE_DEGREE. |
| Arm direction/calibration | IntakeArmSingleServo.java — DIRECTION, FLOOR_OFFSET_DEGREES, SERVO_RANGE_DEGREES; init() PWM range | FORWARD, offset 0, range 300 degrees, PWM 500..2500 microseconds. Command position = (offset + degrees) / 300. These are commanded servo angles, not measured physical arm angles. |

A motor power value is a signed command from -1 to +1. Its sign alone does not universally mean clockwise: motor direction setting and mounting also matter. Shooter velocity percentage selects a speed target; PIDF adjusts motor output to pursue that speed, rather than holding electrical power constant. Encoder ticks/second convert to RPM only with the correct encoder ticks per shaft revolution and gearing.

Arm adjustment step: `TeleopConfig.ARM_STEP_DEGREES = 5.0`. `IntakeArmSingleServo.adjustDegrees()` adjusts the current commanded angle and clamps it to `UP_DEGREES..DOWN_DEGREES`; it does not change those preset constants.

**Hardware configuration names**

| Device | Name required in robot configuration |
|---|---|
| Four drive motors | FLDC, FRDC, RLDC, RRDC |
| Intake DC motor | intake |
| Shooter DC motor with encoder | shooter |
| Feeder DC motor | inputShooterServo — legacy name, still a DC motor, not a servo |
| Intake arm positional servo | intakeServo |
| Autonomous odometry | pinpoint |
| TeleOp hub IMU | imu |
| Autonomous Limelight | limelight |

BioBuzzTeleop currently calls driveTest.init(hardwareMap, false), selecting the hub IMU initialization. Its robot-relative drive() does not use heading. Passing true initializes/resets Pinpoint; it does not by itself enable field-relative driving. Constants.localizerConfig holds Pinpoint offsets X=0.1 inch, Y=2.5 inches, SWINGARM pod type, and both encoder directions REVERSED. These encoder directions are distinct from wheel motor directions.

**Autonomous positions, headings, and sequence timing**

Edit the START, FLOWER_*, and GARDEN_* Waypoint entries in config/AutoConfig.java. Each entry takes X/Y in inches and heading in DEGREES; pose() converts heading to radians. RED and BLUE now share these entries, so each route change affects both alliances. The configured timers and mechanism settings also apply to both alliances.

| Route | Active waypoints (X, Y, heading in degrees) |
|---|---|
| Garden | start (0,0,0); shooting/home (-1,0,0); approach (-4,0,0); reach (-4,-52.90,0); slide-in (-0.7,-52.90,0); slide-out (-1,-54,0); flower start (-4,0,0); flower reached (-26.36,-36.74,270); flower slide-in (-25.97,-43.89,270); parking clearance (-4,-23,0); park (-88,-45,0) |
| Flower | start (0,0,0); FlwrPos1 (-1,0,0); straightEnd (-10,0,0); strafeEnd (-10,10,0); FlwrPos2 (-5,10,0); exitStraight (-11,10,0); finalPose (-11,42,0) |

- TeleOp shooter/feeder start and stop delays: TeleopConfig.SHOOTER_START_DELAY_SECONDS / SHOOTER_STOP_DELAY_SECONDS = 1.0 each, nonblocking.
- Auto feeder start delay: AutoConfig.FEEDER_DELAY_SECONDS = 1.0. 
- Intake and shooter start together; feeder starts afterward. Sequence duration includes this delay.
- Auto shooting durations in AutoConfig: Garden first 4 seconds, second 3, optional third 3; Flower first 5, second 6. All include the 1-second feeder startup delay.
- Garden intake runs during collection travel, the 2-second slide-in wait, return, and hive check. Require two or more alliance Garden tags continuously for 1 second within a 2-second check. Without confirmation, skip second shooting and the flower trip and park. First and third shots have no hive check.
- Garden and Flower deadline: PARK_DEADLINE_SECONDS = 24 and AUTO_LIMIT_SECONDS = 30. At 24 seconds, 
- scoring is interrupted for parking from the live pose if parking has not started (Garden via clearance; Flower directly); 
- arrival within 6 seconds is not guaranteed. At 30 seconds all motion stops.
- Flower hive gate: HIVE_WAIT_TIMEOUT_SECONDS = 10 and HIVE_SETTLE_SECONDS = 3.
- Requires at least two tags continuously for 3 seconds, or shoots at the 10-second overall timeout without confirmation. 
- No second-shot hive check.
- Flower arm-down pause is Delay(1), followed by the backward movement and Delay(2). 
- The backward leg currently uses buildTimedBackwardCommand() with BACKWARD_TEST_SECONDS = 1.0; 
- it does not guarantee reaching the five-inch endpoint. The normal buildMoveCommand() remains commented 
- beside it.
- Flower now also has the 24-second deadline: cancel scoring, stop intake/shooter/feeder, fold the arm up, stop Limelight polling, and drive directly from live pose to finalPose. Normal parking retains its existing two movement legs. An already-running parking route is not restarted at the deadline. Direct recovery paths require field clearance testing.
- HiveUpTagIDLogic uses pipeline 0, 250 ms maximum frame age, and at least two distinct IDs. 
- RED Flower=30..33, 
- RED Garden=34..37, 
- BLUE Flower=38..41, 
- BLUE Garden=42..45. 
- Tag visibility is the current UP status logic, not measured tilt. 
- Both routines stop Limelight polling before parking and on STOP.

**Central Auto timer field names**

All in `AutoConfig`: `GARDEN_FIRST_SHOOT_SECONDS` (6), `GARDEN_SECOND_SHOOT_SECONDS` (4), `GARDEN_COLLECTION_PAUSE_SECONDS` (0.5), `GARDEN_COLLECTION_SECONDS` (2), `GARDEN_HIVE_WAIT_SECONDS` (2), `FLOWER_FIRST_SHOOT_SECONDS` (5), `FLOWER_SECOND_SHOOT_SECONDS` (6), `HIVE_WAIT_TIMEOUT_SECONDS` (10), `HIVE_SETTLE_SECONDS` (3), `FLOWER_ARM_PAUSE_SECONDS` (1), `FLOWER_SECOND_POSITION_PAUSE_SECONDS` (2), `BACKWARD_TEST_SECONDS` (1), `FEEDER_DELAY_SECONDS` (1), `PARK_DEADLINE_SECONDS` (24), and `AUTO_LIMIT_SECONDS` (30). Shooting durations include feeder startup delay and must exceed it. `DEFAULT_SHOOT_SECONDS` (6) is only used by the no-argument shooting command.

**RED/BLUE Garden collection revision:** `AutoConfig.GARDEN_FIRST_SHOOT_SECONDS = 4.0` (total, including feeder startup). After that shot, intake starts before moving toward the garden, remains on through approach/reach/slide-in, runs for `GARDEN_COLLECTION_SECONDS` (2 seconds) at slide-in, and stays running through slide-out, return, and the hive check. Shooter and feeder remain stopped during this trip. Parking deadline and OpMode STOP also stop intake. BLUE now uses the same sequence, coordinates, and timing as RED.

**RED/BLUE Garden second-shot gate:** At shooting home, require at least two fresh alliance Garden tags (RED 34–37; BLUE 42–45) continuously for `GARDEN_HIVE_STABLE_SECONDS` (1 second), within `GARDEN_HIVE_WAIT_SECONDS` (2 seconds). Missing/stale detections reset stability. Without confirmation, skip the second shot and park; there is no shoot-on-timeout fallback for either Garden routine. Intake runs throughout return/check, then through the shot if confirmed; parking stops intake/shooter/feeder and Limelight polling. `GARDEN_PARK` now uses heading 0 degrees. BLUE uses its own Garden tag range, 42–45.

**RED/BLUE Garden optional flower trip:** Only after the confirmed second shot completes, keep intake running and stop shooter/feeder. Move through `GARDEN_FLOWER_START`, lower arm, move through `GARDEN_FLOWER_REACHED` and `GARDEN_FLOWER_SLIDE_IN`, wait `GARDEN_FLOWER_COLLECTION_SECONDS` (2), return to REACHED, fold arm, and return to `GARDEN_SHOOT_HOME`. Shoot for `GARDEN_THIRD_SHOOT_SECONDS` (3 total, including feeder startup) without another hive check, then park. If second shooting is skipped, skip the entire flower trip. The 24-second parking override and 30-second stop remain active. Flower-trip headings are 270 degrees; return heading is 0 degrees.
**RED_AUTO_Garden_NoFlwr:** Separate RED Garden-only OpMode using the shared AutoConfig Garden waypoints and mechanism settings. Move 1 inch, shoot for 4 seconds, collect from Garden (intake stays on during travel and the 2-second collection wait), and return home. Wait for RED Garden tags to be stable for 1 second or for the 2-second timeout; shoot for 3 seconds in either case. Skip the Flower trip entirely, stop mechanisms and Limelight, fold the arm, and park via clearance. All current route headings are 0 degrees. The 24-second parking override and 30-second stop take priority over this sequence. Existing RED_AutoGarden is unchanged.

**BLUE_AUTO_Garden_NoFlwr:** BLUE counterpart of RED_AUTO_Garden_NoFlwr with the same shared Garden coordinates, powers, timings, and parking deadlines. Uses BLUE Garden AprilTags (42-45). Shoots after stable confirmation or the 2-second timeout, then parks without Flower collection. Place the robot at the corresponding BLUE start, physically rotated 180 degrees from RED, and initialize local pose to (0,0,0).

**RED_AUTO_GARD_PrkOnly / RED_AUTO_FLWR_PrkOnly:** Drive-only parking modes. INIT prepares Pedro/odometry and paths; START immediately drives from START through GARDEN_BEFOREPARK_STRAFE to GARDEN_PARK, or through FLOWER_EXIT_STRAIGHT to FLOWER_PARK, respectively. The first leg is a straight segment from the starting pose to that intermediate waypoint; validate its clearance on the field. Uses AutoConfig coordinates and MAX_PATH_SPEED; stops at AUTO_LIMIT_SECONDS (30). No shooter, feeder, intake, arm, or Limelight hardware is initialized. Position the arm appropriately before running. Existing scoring modes remain available.

**BLUE_AUTO_GARD_PrkOnly / BLUE_AUTO_FLWR_PrkOnly:** BLUE drive-only counterparts of the RED parking-only modes, using the same shared Garden/Flower intermediate and parking waypoints. Place the robot at the corresponding BLUE start with orientation rotated 180 degrees from RED; INIT sets local pose to (0,0,0). START immediately runs the parking path. Only drive/odometry is initialized, with the same 30-second overall stop and no scoring or vision hardware.

**Waypoint timeout (all ten main AutoAppl modes):** AutoConfig.WAYPOINT_TIMEOUT_SECONDS = 3.0 starts separately for each buildMoveCommand. On timeout the follower stops and the sequence advances, including a following collection or shooting action even if the target was not reached. Each next waypoint path starts at live odometry. Telemetry preserves the last timeout and a timeout count. Shooting, intake waits, hive waits, and arm delays are unchanged. Flower's existing timed backward test retains its own duration. No new position tolerance, rotation/freeze watchdog, or sensor-health checks were added. Existing overall deadlines still apply. Timeout is not proof of arrival; validate routes before relying on automatic continuation.

**Gamepad 2 shooter triggers:** LT above 50% starts the existing shooter-then-feeder sequence once per press; release does not stop shooting. RT above 50% stops feeder then shooter using the existing stop delay and wins if both triggers are pressed. Trigger depth does not set speed; A/B remain speed controls. Controller START/BACK no longer start/stop shooting. Triggers held during INIT/Driver Station START do not initiate shooting until released and pressed again.

### TeleOp joystick sensitivity
Forward/backward and strafe use DRIVE_DEADBAND = 0.05 and DRIVE_RESPONSE_EXPONENT = 2.0 in config/TeleopConfig.java. Set exponent to 1.0 for linear response. DRIVE_NORMALIZATION_FLOOR = 1.0 preserves full stick travel before the DRIVE_MAX_POWER = 0.25 cap. Turning keeps its squared input curve; TURN_MAX_POWER = 0.10 sets the maximum wheel power when turning alone, capped by DRIVE_MAX_POWER. Combined translation and turning are normalized. No acceleration ramp is applied; releasing the stick commands zero immediately.

### Shooter speed button mapping update
Gamepad 2 D-pad RIGHT increases shooter target speed by 5 percentage points per press; D-pad LEFT decreases it. These replace A/B speed controls. Holding changes once; simultaneous LEFT+RIGHT is ignored. LT/RT shooting sequence and D-pad UP/DOWN arm adjustment remain unchanged.

### Shooter shot calibration CSV
Select **Shooter Shot Calibration** (Calibration group). This separate TestAppl OpMode inherits BioBuzzTeleop controls, including motor/servo initialization and arm UP at INIT. It also initializes Pinpoint with the autonomous configuration and starts Limelight pipeline 0. Keep the robot still during INIT. START sets the local odometry origin to (0,0,0); coordinates from separate sessions need the same physical starting reference to be compared.

Drive and shoot normally. After a successful shot, keep the robot at that position and press **Gamepad 1 A**. One new press saves one CSV row; holding A does not repeat. No samples are taken during INIT. Gamepad 2 D-pad LEFT/RIGHT still adjusts shooter speed; LT starts shooter then feeder and RT stops them using the existing delays. This logger does not perform automatic aiming, hive classification or shot detection.

CSV files are created at INIT in `/sdcard/FIRST/logs/shooter-calibration/`. The full filename and successful capture count appear in telemetry. Each capture is flushed immediately; STOP closes the file and stops the camera. Missing camera/odometry readings are marked; camera results older than 250 ms, invalid results or wrong-pipeline results do not populate tag measurements. Raw odometry availability does not establish sensor health or stationary status. A hard power loss can still lose data despite flushing.

Columns include sample ID, host timestamp (hub clock may be wrong), monotonic elapsed seconds, operator-reported success, manual attempts/hits/notes, local robot pose in inches/degrees, selected shooter speed percentage, current target/measured encoder velocity, SDK power command, minimum positive reported hub voltage, camera status/frame age and visible IDs. Tags 30 through 45 each have their own camera-relative XYZ in meters, roll/pitch/yaw and tx/ty in degrees. Blank 3D fields mean absent/unusable pose; tag size and Full 3D targeting must be configured for meaningful distance. These are raw tag poses, not corrected shooter-to-hive-opening distances.

All measurements are **at A press**, not at ball release. If the shooter was stopped before capture, target/measured speed may be zero; selected speed remains recorded. For a batch of 10 attempts with 3 hits, record attempts=10 and hits=3 manually on a representative row and avoid counting the batch repeatedly on every success row. No misses are automatically inferred.

With the laptop connected to Control Hub Wi-Fi, retrieve using Android SDK platform-tools (adb):
```text
adb connect 192.168.43.1:5555
adb pull /sdcard/FIRST/logs/shooter-calibration ./shooter-calibration
```
With multiple adb devices, select the hub with `adb -s 192.168.43.1:5555 pull ...`. CSV files open in Excel or Google Sheets. Update calibration files only after copying them off the hub.
Shot calibration alliance selection: during INIT, Gamepad 1 LB selects RED (IDs 30-37), RB selects BLUE (IDs 38-45). Default is RED. START locks the choice for that run. CSV includes an alliance column; opposing and non-hive IDs are omitted from visible IDs, live tag telemetry and pose fields. If only opposing tags are visible, A still saves the operator marker and shooter/odometry readings but camera_status is NO_ALLIANCE_TAG and all tag fields are blank. Select STOP/INIT to change alliance. Both alliances use the same calibration OpMode.

### Current dual-shooter TeleOp controls
PollenShooter replaces ShooterBBTeleop; its hardware name remains `shooter`. Autonomous still initializes only PollenShooter with unchanged velocity behavior. NectarShooter is TeleOp-only (including the inherited Shooter Shot Calibration test) and requires the encoder-equipped motor named `nectarShooter`.

Gamepad 2: LT starts pollen and nectar together, then the shared feeder after SHOOTER_START_DELAY_SECONDS (1 second). RT stops the feeder immediately, then both shooters after SHOOTER_STOP_DELAY_SECONDS (1 second). Holding RT does not restart the stop timer. OpMode STOP stops all three immediately. The start delay is not an actual-velocity readiness check.

D-pad RIGHT/LEFT increases/decreases pollen target speed; UP/DOWN increases/decreases nectar target speed. Each press changes 2 percentage points, holding does not repeat, opposite buttons on the same axis are ignored. Setting either shooter speed to zero stops both shooters and the feeder; LT cannot start until both selected speeds are positive. Raising speed while stopped does not start the motors. Gamepad 2 no longer adjusts the intake arm; Gamepad 1 UP/DOWN retains arm presets. Gamepad 2 X retains intake reversal.

TeleopConfig: INITIAL_SHOOTER_SPEED_PERCENT=60, MAX_SHOOTER_SPEED_PERCENT=85, SHOOTER_STEP_PERCENT=2; INITIAL_NECTAR_SPEED_PERCENT=60, MAX_NECTAR_SPEED_PERCENT=85, NECTAR_STEP_PERCENT=2. Both are velocity percentages, not fixed power. Each shooter class has its own MAX_VELOCITY_TICKS_PER_SECOND=2600 and motor direction REVERSE, initially matching the original shooter. Both use RUN_USING_ENCODER with the hub/configured motor's velocity PIDF; no new custom PIDF coefficients were added. Verify the nectar encoder, configured motor type, direction and attainable speed on hardware.

Shooter Shot Calibration inherits both motors and records nectar selected/target/measured velocity and SDK power in additional CSV columns. Existing unprefixed shooter CSV columns continue to describe pollen.
### Continuous intake helper
Configure a continuous-rotation servo named exactly IntakeHelperServo. Program the physical servo for continuous mode. IntakeHelperServo.java defines RUN_POWER=0.80 and DIRECTION=FORWARD; reverse DIRECTION if normal operation runs outward. Intake initializes both actuators stopped, starts the helper at fixed 80% for any nonzero intake DC command, reverses it with intake reversal, and stops it when intake stops or its selected power is zero. DC speed settings and existing autonomous/TeleOp sequencing are unchanged. Every OpMode initializing Intake now requires this servo mapping; servo-only arm tests do not initialize it.

### TeleOp intake helper adjustment
Gamepad 2 A increases helper servo power by 2 percentage points; B decreases it by 2. Range is 0-100%, default 80% each INIT. Hold does not repeat; A+B and START/BACK pairing chords are ignored. Changes affect a running helper immediately on the next intake command; adjusting while stopped does not start it. Gamepad 1 RIGHT/LEFT still starts/stops both intake actuators; RIGHT resets DC power only and retains helper selection. Reversal still applies to both. Autonomous retains the helper's default 80%. TeleopConfig defines INITIAL_INTAKE_HELPER_POWER_PERCENT and INTAKE_HELPER_STEP_PERCENT.
