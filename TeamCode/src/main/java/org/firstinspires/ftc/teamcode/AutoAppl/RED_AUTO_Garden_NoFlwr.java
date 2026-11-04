/* ============================================================================
 * Class: RED_AUTO_Garden_NoFlwr
 * What it does:
 *   Shoots, collects from RED Garden, shoots on confirmation or timeout, then parks without the Flower trip.
 * Applications using it:
 *   RED_AUTO_Garden_NoFlwr (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.AutoAppl;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoShootSequence;
import org.firstinspires.ftc.teamcode.subsystems.IntakeArmSingleServo;
import org.firstinspires.ftc.teamcode.subsystems.HiveUpTagIDLogic;
import dev.nextftc.core.commands.utility.InstantCommand;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RED_AUTO_Garden_NoFlwr")
public class RED_AUTO_Garden_NoFlwr extends NextFTCOpMode {

    private String lastMovementResult = "None";
    private String lastWaypointTimeout = "None";
    private int waypointTimeoutCount;
    private Follower follower;
    private Command routine, deadlinePark;
    private long autoStartedAt;
    private boolean autoRunning, parkingStarted;
    private String autoStatus = "Ready";
    private final AutoShootSequence shooting = new AutoShootSequence();
    private Pose startPose;
    private Pose shootingPose;
    private Pose gardenApproach;
    private Pose gardenPos;
    private final IntakeArmSingleServo arm = new IntakeArmSingleServo();
    private HiveUpTagIDLogic hiveTags;
    private long hiveWaitStarted;
    private boolean hiveReady, hiveCheckDone;
    private long hiveStableSince;
    private String hiveResult = "First shot: no check required";
    private Pose strafeClearancePose;
    private Pose gardenSlideIn, gardenSlideOut, shootingHome;
    private Pose turn1;

    public RED_AUTO_Garden_NoFlwr() {
        // PedroComponent / NextFTC Pedro extension was for Pedro 2.
        // With Pedro 3, own the Follower directly and call follower.update() in onUpdate().
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    /**
     * Pedro 3 + NextFTC v1 replacement for the old FollowPath command.
     *
     * Old code:
     *   new FollowPath(pathChain, false, maximumPower)
     *
     * Pedro 3 now follows a Path directly with follower.follow(path).
     * LambdaCommand keeps it compatible with the existing NextFTC SequentialGroup.
     */
    private Command buildMoveCommand(Pose fromPosition, Pose toPosition) {
        final long[] movementStartedAt = {0};
        final boolean[] timedOut = {false};
        final String movementLabel = "Waypoint to " + toPosition.x() + ", " + toPosition.y();
        return new LambdaCommand(movementLabel)
                .setStart(() -> {
                    autoStatus = movementLabel;
                    timedOut[0] = false;
                    // Previous leg may have timed out: use measured pose instead of its target.
                    Pose actualStart = follower.pose();
                    follower.follow(line(actualStart, toPosition).linear(actualStart, toPosition));
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
    private void buildPath() {
        startPose = START.pose();
        shootingPose = GARDEN_SHOOT.pose();
        gardenApproach = GARDEN_APPROACH_START.pose();
        gardenPos = GARDEN_COLLECT_REACH.pose();
        gardenSlideIn = GARDEN_COLLECT_SLIDE_IN.pose();
        gardenSlideOut = GARDEN_COLLECT_SLIDE_OUT.pose();
        shootingHome = GARDEN_SHOOT_HOME.pose();
        strafeClearancePose = GARDEN_BEFOREPARK_STRAFE.pose();
        // Zero-degree parking heading: maintain the starting orientation.
        turn1 = GARDEN_PARK.pose();
    }
    /** Wait for stable RED Garden tags, or shoot after the configured timeout without confirmation. */
    private Command buildGardenHiveWaitCommand() {
        return new LambdaCommand("Confirm Garden hive before second shot")
                .setStart(() -> {
                    // Intake keeps running from collection; shooter and feeder are still stopped.
                    hiveTags.reset();
                    hiveWaitStarted = System.nanoTime();
                    hiveStableSince = 0;
                    hiveReady = false;
                    hiveCheckDone = false;
                    hiveResult = "Checking Garden UP stability";
                })
                .setUpdate(() -> {
                    hiveTags.update();
                    long now = System.nanoTime();
                    if (hiveTags.isRedGardenUp()) {
                        if (hiveStableSince == 0) hiveStableSince = now;
                        double stableSeconds = (now - hiveStableSince) / 1e9;
                        hiveReady = stableSeconds >= GARDEN_HIVE_STABLE_SECONDS;
                        hiveResult = hiveReady ? "Garden UP confirmed: shoot"
                                : String.format("Garden UP settling: %.2f / %.2f s",
                                        stableSeconds, GARDEN_HIVE_STABLE_SECONDS);
                    } else {
                        hiveStableSince = 0;
                        hiveReady = false;
                        hiveResult = "Garden not confirmed UP; stability reset";
                    }
                    hiveCheckDone = hiveReady;
                    if (!hiveReady && (now - hiveWaitStarted) / 1e9 >= GARDEN_HIVE_WAIT_SECONDS) {
                        hiveCheckDone = true;
                        hiveResult = "Hive check timed out: shoot without confirmation, then park";
                    }
                })
                .setIsDone(() -> hiveCheckDone);
    }
    private void prepareParking() {
        parkingStarted = true;
        shooting.stop();
        arm.liftUp();
        hiveTags.stop();
        autoStatus = "Parking";
    }

    /** Interrupt scoring at 24 s; use live odometry, never an assumed completed waypoint. */
    private void enforceAutoDeadline() {
        if (!autoRunning) return;
        double elapsed = (System.nanoTime() - autoStartedAt) / 1e9;
        if (elapsed >= AUTO_LIMIT_SECONDS) {
            autoRunning = false;
            if (routine != null) routine.cancel();
            if (deadlinePark != null) deadlinePark.cancel();
            shooting.stop();
            hiveTags.stop();
            follower.stop();
            autoStatus = AUTO_LIMIT_SECONDS + " seconds: all motion stopped";
        } else if (elapsed >= PARK_DEADLINE_SECONDS && !parkingStarted) {
            if (routine != null) routine.cancel();
            follower.stop();
            prepareParking();
            autoStatus = PARK_DEADLINE_SECONDS + "-second deadline: parking via clearance";
            // Use the clearance waypoint even on timeout; validate approach from collection positions.
            deadlinePark = new SequentialGroup(
                    buildMoveCommand(follower.pose(), strafeClearancePose),
                    buildMoveCommand(strafeClearancePose, turn1),
                    new InstantCommand(() -> autoStatus = "Parking path complete"));
            deadlinePark.schedule();
        }
    }
    private Command autonomousRoutine() {
        return new SequentialGroup(
                buildMoveCommand(startPose, shootingPose),
                shooting.command(SHOOTER_SPEED_FRACTION, GARDEN_FIRST_SHOOT_SECONDS),
                // Start intake before leaving the shooting position; keep it on during travel.
                new InstantCommand(shooting::startCollecting),
                buildMoveCommand(shootingPose, gardenApproach),
                buildMoveCommand(gardenApproach, gardenPos),
                buildMoveCommand(gardenPos, gardenSlideIn),
                // Keep intake running at slide-in and throughout the return and hive check.
                new Delay(GARDEN_COLLECTION_SECONDS),
                buildMoveCommand(gardenSlideIn, gardenSlideOut),
                buildMoveCommand(gardenSlideOut, shootingHome),
                buildGardenHiveWaitCommand(),
                // Total duration includes the configured feeder startup delay.
                // Always shoot after the wait, even without hive confirmation. No Flower trip.
                shooting.command(SHOOTER_SPEED_FRACTION, GARDEN_SECOND_SHOOT_SECONDS),
                new InstantCommand(this::prepareParking),
                buildMoveCommand(shootingHome, strafeClearancePose),
                buildMoveCommand(strafeClearancePose, turn1),
                new InstantCommand(() -> autoStatus = "Parking path complete")
        );
    }
    @Override
    public void onInit() {
        autoRunning = false;
        parkingStarted = false;
        deadlinePark = null;
        buildPath();
        shooting.init(hardwareMap); // Hardware ready but stopped during INIT.
        arm.init(hardwareMap); // Keep the arm folded for this Garden-only route.
        hiveTags = new HiveUpTagIDLogic(hardwareMap);
        hiveTags.start();

        // Keep createFollower() if that is the method your Constants.java currently provides.
        // The current Pedro 3 docs often name this user-defined helper create().
        follower = Constants.createFollower(hardwareMap);

        // Pedro 3 uses setPose(); setStartingPose() is no longer needed.

        follower.setPose(startPose);
        routine = autonomousRoutine();

    }

    @Override
    public void onUpdate() {
        // REQUIRED: Pedro 3 follower must be updated every OpMode loop.
        follower.update();
        enforceAutoDeadline();
        telemetry.addData("Auto status", autoStatus);
        telemetry.addData("Auto seconds remaining", "%.1f", autoRunning
                ? Math.max(0, AUTO_LIMIT_SECONDS - (System.nanoTime() - autoStartedAt) / 1e9) : 0);

        telemetry.addData("Shooting sequence", shooting.getStatus());
        telemetry.addData("Garden hive check", hiveResult);
        telemetry.addData("Intake arm target (deg)", arm.getCommandedDegrees());
        telemetry.addData("Garden tag IDs", hiveTags.getGardenTags());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower mode", follower.mode());
        telemetry.addData("Last movement result", lastMovementResult);
        telemetry.addData("Waypoint timeouts", waypointTimeoutCount);
        telemetry.addData("Last waypoint timeout", lastWaypointTimeout);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autoStartedAt = System.nanoTime();
        autoRunning = true;
        autoStatus = "Scoring sequence";
        routine.schedule();
    }

    @Override
    public void onStop() {
        autoRunning = false;
        if (routine != null) routine.cancel();
        if (deadlinePark != null) deadlinePark.cancel();
        if (hiveTags != null) hiveTags.stop();
        shooting.stop();
        if (follower != null) {
            follower.stop();
        }
    }
}
