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
| START | Start shooter at selected velocity; after 1 second, start feeder (InputDCShooter). Requires a fresh press after Driver Station START. |
| BACK | Stop feeder immediately; stop shooter 1 second later. Overrides START.                                                                |
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
| TeleOp turning | config/TeleopConfig.java — TURN_DEADBAND, MAX_TURN_INPUT (response calculation remains in BioBuzzTeleop.loop()) | 0.05 deadband, 0.20 maximum input, squared response. Wheel mixing/cap still applies. |
| Auto drive limit | config/AutoConfig.java — MAX_PATH_SPEED (read by Pedro Constants) | 0.80 Pedro path-speed setting; not a fixed wheel power or inches/second. Auto does not use MecanumDrive.drive(). |
| Auto wheel direction/names | Constants.java — drivetrainConfig: frontLeft/backLeft/frontRight/backRight Name and Direction | FLDC/RLDC FORWARD; FRDC/RRDC REVERSE. Separate settings from TeleOp; keep both configurations consistent with wiring. |
| TeleOp intake initial power | config/TeleopConfig.java — INITIAL_INTAKE_POWER_PERCENT, MAX_INTAKE_POWER_PERCENT, INTAKE_STEP_PERCENT | 90, reset during INIT and whenever D-pad RIGHT starts intake. Gamepad 1 X subtracts 10 and Y adds 10 in loop(), bounded to 0..100%. |
| Auto intake power | config/AutoConfig.java — INTAKE_POWER | 1.00 (100%), used during shooting and collection by all four main Auto routines. |
| Intake direction | subsystems/Intake.java — start(double powerFraction, boolean reverse) | Normal sends negative power; reverse sends positive power. init() maps "intake" but does not explicitly set motor direction or run mode. |
| Feeder power | config/AutoConfig.java and config/TeleopConfig.java — FEEDER_POWER | -0.50 in each config, independently selected during initialization. setPower(...) stores a value for the next start(); it does not immediately update an already-running motor. |
| Feeder direction/mode | InputDCShooter.init() | FORWARD direction, RUN_WITHOUT_ENCODER, BRAKE. Physical rotation combines direction with signed power. |
| Shooter initial/max selection | config/TeleopConfig.java — INITIAL_SHOOTER_SPEED_PERCENT / MAX_SHOOTER_SPEED_PERCENT / SHOOTER_STEP_PERCENT | 60 / 85. SHOOTER_STEP_PERCENT = 5 controls A/B increments. This 85% cap belongs to TeleOp. |
| Auto shooter selection | config/AutoConfig.java — SHOOTER_SPEED_FRACTION | 0.60. Each current Auto passes this into shooting.command(...). Auto does not read TeleOp's selected percentage or its 85% cap. |
| Shooter velocity reference | subsystems/ShooterBBTeleop.java — MAX_VELOCITY_TICKS_PER_SECOND | 2600 ticks/second; not RPM and not a measured physical maximum. 60%=1560; 85%=2210; 5 percentage points=130 ticks/second. |
| Shooter direction | ShooterBBTeleop.initialize() — shooterMotor.setDirection(...) | REVERSE, with positive velocity targets. |
| Shooter feedback/mode | ShooterBBTeleop.initialize(), runVelocity() | RUN_USING_ENCODER and setVelocity(), using REV Hub velocity PIDF/configured motor type. No custom kP/kI/kD values are assigned here. FLOAT on stop. |
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