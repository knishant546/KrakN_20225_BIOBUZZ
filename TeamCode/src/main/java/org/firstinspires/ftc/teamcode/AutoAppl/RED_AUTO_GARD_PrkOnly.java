/* ============================================================================
 * Class: RED_AUTO_GARD_PrkOnly
 * What it does: Drives from the RED Garden start to parking via the shared clearance
 *   waypoint. Uses odometry only; no shooter, feeder, intake, arm, or Limelight.
 * Applications using it: RED_AUTO_GARD_PrkOnly (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.AutoAppl;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RED_AUTO_GARD_PrkOnly")
public class RED_AUTO_GARD_PrkOnly extends NextFTCOpMode {
    private String lastMovementResult = "None";
    private String lastWaypointTimeout = "None";
    private int waypointTimeoutCount;
    private Follower follower;
    private Command routine;
    private long startedAt;
    private boolean running;
    private String stage = "Ready";

    public RED_AUTO_GARD_PrkOnly() {
        addComponents(BulkReadComponent.INSTANCE);
    }

    private Command buildMoveCommand(Pose from, Pose to, String label) {
        final long[] movementStartedAt = {0};
        final boolean[] timedOut = {false};
        final String movementLabel = label;
        return new LambdaCommand(movementLabel)
                .setStart(() -> {
                    stage = movementLabel;
                    timedOut[0] = false;
                    // Previous leg may have timed out: use measured pose instead of its target.
                    Pose actualStart = follower.pose();
                    follower.follow(line(actualStart, to).linear(actualStart, to));
                    movementStartedAt[0] = System.nanoTime();
                })
                .setIsDone(() -> {
                    if (!follower.isBusy()) return true;
                    timedOut[0] = (System.nanoTime() - movementStartedAt[0]) / 1e9
                            >= WAYPOINT_TIMEOUT_SECONDS;
                    return timedOut[0];
                })
                .setStop(interrupted -> {
                    follower.stop();
                    // Timeout advances the sequence, including any following mechanism action.
                    lastMovementResult = movementLabel + (interrupted ? ": interrupted"
                            : timedOut[0] ? ": TIMEOUT - advancing" : ": follower completed");
                    if (timedOut[0] && !interrupted) {
                        waypointTimeoutCount++;
                        lastWaypointTimeout = lastMovementResult;
                    }
                });
    }

    @Override public void onInit() {
        running = false;
        follower = Constants.createFollower(hardwareMap);
        Pose start = START.pose();
        Pose clearance = GARDEN_BEFOREPARK_STRAFE.pose();
        Pose park = GARDEN_PARK.pose();
        follower.setPose(start);
        // Build during INIT; START schedules motion immediately, with no scoring delay.
        routine = new SequentialGroup(
                buildMoveCommand(start, clearance, "Moving to parking clearance"),
                buildMoveCommand(clearance, park, "Moving to park"),
                new InstantCommand(() -> {
                    follower.stop();
                    running = false;
                    stage = "Parking path complete";
                }));
        stage = "Ready: START to park; mechanisms are not initialized";
    }

    @Override public void onStartButtonPressed() {
        startedAt = System.nanoTime();
        running = true;
        routine.schedule();
    }

    @Override public void onUpdate() {
        follower.update();
        double elapsed = running ? (System.nanoTime() - startedAt) / 1e9 : 0;
        // Already parking from START; only the overall Auto limit is needed.
        if (running && elapsed >= AUTO_LIMIT_SECONDS) {
            routine.cancel();
            follower.stop();
            running = false;
            stage = "Auto time limit: drive stopped";
        }
        telemetry.addData("Stage", stage);
        telemetry.addData("Running", running);
        telemetry.addData("X (in)", follower.pose().x());
        telemetry.addData("Y (in)", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Last movement result", lastMovementResult);
        telemetry.addData("Waypoint timeouts", waypointTimeoutCount);
        telemetry.addData("Last waypoint timeout", lastWaypointTimeout);
        telemetry.update();
    }

    @Override public void onStop() {
        running = false;
        if (routine != null) routine.cancel();
        if (follower != null) follower.stop();
    }
}