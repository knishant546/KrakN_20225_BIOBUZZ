# Configuration reference

Snapshot from Java assignments, November 16, 2026. Java source remains authoritative. Edit constants, rebuild and install; these are not live dashboard settings.

AutoConfig supplies autonomous power, routes and timers. TeleopConfig supplies driver controls and the shared ShooterGateControl readiness/recovery thresholds. Shooter gate angles and PWM are in subsystems/ShooterGateServo.java; intake arm calibration is in subsystems/IntakeArmSingleServo.java. Motor names/directions are in their subsystem init methods; Pedro drive/localization settings are under subsystems/pedropathing.

Percentages for Pollen are velocity-reference percentages, not motor power. Feeder and intake use signed DC power. Current Auto/TeleOp feeder callers negate FEEDER_POWER to preserve the tested direction. Coordinates are local inches and headings are degrees, converted to radians by Waypoint.pose(). BLUE uses the same local path after a 180-degree physical starting rotation.

Shot durations start at gate OPEN. Startup feeder fallback is 1.5 seconds; speed recovery has no repeated timeout bypass. Auto adds 0.5 seconds gate-opening dwell. Legacy delay/helper/nectar settings retained for older code do not enable those features in the current routines.

## AutoConfig

| Constant | Assigned value |
|---|---|
| `SHOOTER_SPEED_FRACTION` | `0.60` |
| `INTAKE_POWER` | `0.90` |
| `FEEDER_POWER` | `-0.50` |
| `MAX_PATH_SPEED` | `1.00` |
| `GATE_OPEN_DELAY_SECONDS` | `0.5` |
| `FEEDER_DELAY_SECONDS` | `1.0` |
| `SHOOTER_TO_INTAKE_DELAY_SECONDS` | `2.0` |
| `DEFAULT_SHOOT_SECONDS` | `6.0` |
| `PARK_DEADLINE_SECONDS` | `24.0` |
| `WAYPOINT_TIMEOUT_SECONDS` | `3.0` |
| `GARDEN_SHOT_POSITION_TOLERANCE_INCHES` | `0.5` |
| `GARDEN_SHOT_HEADING_TOLERANCE_DEGREES` | `3.0` |
| `AUTO_LIMIT_SECONDS` | `30.0` |
| `GARDEN_FULL_FIRST_SHOOT_SECONDS` | `3.0` |
| `GARDEN_REACH_TIMEOUT_SECONDS` | `2.0` |
| `GARDEN_SLIDE_OUT_TIMEOUT_SECONDS` | `2.0` |
| `GARDEN_FLOWER_BACKOUT_TIMEOUT_SECONDS` | `2.0` |
| `GARDEN_FLOWER_MIN_REMAINING_SECONDS` | `14.0` |
| `GARDEN_FIRST_SHOOT_SECONDS` | `5.0` |
| `GARDEN_NOFLWR_SECOND_SHOOT_SECONDS` | `5.0` |
| `GARDEN_SECOND_SHOOT_SECONDS` | `3.0` |
| `GARDEN_COLLECTION_PAUSE_SECONDS` | `0.5` |
| `GARDEN_COLLECTION_SECONDS` | `2.0` |
| `GARDEN_HIVE_WAIT_SECONDS` | `2.0` |
| `GARDEN_HIVE_STABLE_SECONDS` | `1.0` |
| `GARDEN_SLIDE_IN_SECONDS` | `1.0` |
| `GARDEN_FLOWER_COLLECTION_SECONDS` | `2.0` |
| `GARDEN_THIRD_SHOOT_SECONDS` | `3.0` |
| `FLOWER_TRIP_MIN_REMAINING_SECONDS` | `15.0` |
| `FLOWER_FIRST_SHOOT_SECONDS` | `5.0` |
| `FLOWER_SECOND_SHOOT_SECONDS` | `6.0` |
| `HIVE_WAIT_TIMEOUT_SECONDS` | `7.0` |
| `HIVE_TAG_DROPOUT_MS` | `150` |
| `HIVE_SETTLE_SECONDS` | `3.0` |
| `FLOWER_ARM_PAUSE_SECONDS` | `1.0` |
| `FLOWER_POST_SHOT_COAST_SECONDS` | `0.0` |
| `FLOWER_COLLECTION_SECONDS` | `2.0` |
| `FLOWER_SECOND_POSITION_PAUSE_SECONDS` | `0.0` |
| `BACKWARD_TEST_SECONDS` | `1.0` |
| `START` | `new Waypoint(0, 0, 0)` |
| `FLOWER_SHOOT_1` | `new Waypoint(-1, 0, 0)` |
| `FLOWER_STRAIGHT_END` | `new Waypoint(-10, 0, 0)` |
| `FLOWER_STRAFE_END` | `new Waypoint(-10, 10, 0)` |
| `FLOWER_SHOOT_2` | `new Waypoint(-2.5, 10, 0)` |
| `FLOWER_EXIT_STRAIGHT` | `new Waypoint(-11, 10, 0)` |
| `FLOWER_PARK` | `new Waypoint(-11, 42, 0)` |
| `GARDEN_SHOOT` | `new Waypoint(-1, 0, 0)` |
| `GARDEN_APPROACH_START` | `new Waypoint(-4, 0, 0)` |
| `GARDEN_COLLECT_REACH` | `new Waypoint(-4, -52.90, 0)` |
| `GARDEN_COLLECT_SLIDE_IN` | `new Waypoint(-0.7, -52.90, 0)` |
| `GARDEN_COLLECT_SLIDE_OUT` | `new Waypoint(-1, -54.0, 0)` |
| `GARDEN_SHOOT_HOME` | `new Waypoint(-1, 0, 0)` |
| `GARDEN_CLEARANCE` | `new Waypoint(-1, -3, 0)` |
| `RED_GARDEN_EXTRA_CLEARANCE` | `new Waypoint(-1, -6, 0)` |
| `GARDEN_PARK` | `new Waypoint(-73.16, -45.13, -91.24)` |
| `GARDEN_FLOWER_START` | `new Waypoint(-4, 0, 0)` |
| `GARDEN_FLOWER_REACHED` | `new Waypoint(-26.36, -36.74, 270)` |
| `GARDEN_FLOWER_SLIDE_IN` | `new Waypoint(-25.97, -43.89, 270)` |
| `GARDEN_FLOWER_BACKOUT` | `new Waypoint(-26.36, -35.74, 270)` |
| `GARDEN_BEFOREPARK_STRAFE` | `new Waypoint(-4, -23, 0)` |
| `GARDEN_FULL_BEFOREPARK_STRAFE` | `new Waypoint(-4, -23, 0)` |
| `GARDEN_PARK` | `new Waypoint(-88.00, -45, 0)` |

## TeleopConfig

| Constant | Assigned value |
|---|---|
| `INITIAL_SHOOTER_SPEED_PERCENT` | `60` |
| `MAX_SHOOTER_SPEED_PERCENT` | `85` |
| `SHOOTER_STEP_PERCENT` | `1` |
| `INITIAL_NECTAR_SPEED_PERCENT` | `58` |
| `MAX_NECTAR_SPEED_PERCENT` | `85` |
| `NECTAR_STEP_PERCENT` | `1` |
| `FEEDER_POWER` | `-0.50` |
| `INITIAL_INTAKE_POWER_PERCENT` | `90` |
| `MAX_INTAKE_POWER_PERCENT` | `100` |
| `INTAKE_STEP_PERCENT` | `1` |
| `INITIAL_INTAKE_HELPER_POWER_PERCENT` | `80` |
| `INTAKE_HELPER_STEP_PERCENT` | `2` |
| `DRIVE_MAX_POWER` | `0.50` |
| `DRIVE_NORMALIZATION_FLOOR` | `1.0` |
| `DRIVE_DEADBAND` | `0.05` |
| `DRIVE_RESPONSE_EXPONENT` | `2.0` |
| `TURN_DEADBAND` | `0.05` |
| `TURN_MAX_POWER` | `0.10` |
| `ARM_STEP_DEGREES` | `5.0` |
| `SHOOTER_START_TIMEOUT_SECONDS` | `1.5` |
| `SHOOTER_READY_ERROR_FRACTION` | `0.05` |
| `SHOOTER_READY_MAX_ERROR_TICKS` | `200.0` |
| `SHOOTER_DROP_FRACTION` | `0.10` |
| `SHOOTER_DROP_HOLD_SECONDS` | `0.10` |
| `SHOOTER_RECOVERY_HOLD_SECONDS` | `0.20` |
| `SHOOTER_START_DELAY_SECONDS` | `1.0` |
| `SHOOTER_STOP_DELAY_SECONDS` | `1.0` |
