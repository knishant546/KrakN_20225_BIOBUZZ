package org.firstinspires.ftc.teamcode.pedroPathing;

/**
 * Pedro Pathing 3 migration placeholder.
 *
 * The old Pedro 2 Tuning.java used:
 *   - com.pedropathing.telemetry.SelectableOpMode
 *   - com.pedropathing.util.*
 *   - PanelsConfigurables / PanelsTelemetry
 *   - old Follower methods such as setTeleOpDrive(), getPose(), followPath(), etc.
 *
 * Those APIs belong to the Pedro 2 tuning system and should not be carried into
 * a Pedro 3 project.
 *
 * Pedro 3 tuning is performed with AutoTune, which is served by the Robot
 * Controller on port 10158. This placeholder intentionally contains no Pedro 2
 * APIs so the rest of the project can compile while the robot code is migrated.
 *
 * When you are ready to tune the robot, replace this placeholder with the
 * Pedro 3 AutoTune registration class for:
 *   - MecanumTuner
 *   - PinpointTuner
 *   - ForesightTuner
 *
 * Do NOT restore the old 1,300-line Pedro 2 Tuning.java.
 */
public final class Tuning {

    private Tuning() {
        // Utility/registration class -- no instances.
    }
}
