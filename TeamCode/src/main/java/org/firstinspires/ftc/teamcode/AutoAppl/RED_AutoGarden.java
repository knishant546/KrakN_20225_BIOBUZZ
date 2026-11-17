/* ============================================================================
 * Class: RED_AutoGarden
 * What it does:
 *   Runs the RED Garden shooting/collection route, with a 24-second parking deadline and 30-second stop.
 * Applications using it:
 *   RED_AutoGarden (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.AutoAppl;

import org.firstinspires.ftc.teamcode.Logging.RunTelemetryLogger;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoShootSequence;
import org.firstinspires.ftc.teamcode.subsystems.WaypointSettleGuard;
import org.firstinspires.ftc.teamcode.subsystems.IntakeArmSingleServo;
import org.firstinspires.ftc.teamcode.subsystems.HiveUpTagIDLogic;
import dev.nextftc.core.commands.utility.InstantCommand;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "RED_AutoGarden")
public class RED_AutoGarden extends NextFTCOpMode {
    private RunTelemetryLogger runLog = new RunTelemetryLogger();

    // Pedro 3 recommended pose creation API.
    private final PoseFactory p = PoseFactory.degrees();

    private Pose startPoseStraight;
    private Pose adjustPoseToShoot;

    private Pose moveToPickRow;
    private Pose adjustOut;

    private String lastMovementResult = "None";
    private String lastWaypointTimeout = "None";
    private int waypointTimeoutCount;
    private Follower follower;
    private Command routine, deadlinePark;
    private long autoStartedAt;
    private boolean autoRunning, parkingStarted;
    private String autoStatus = "Ready";
    private final AutoShootSequence shooting = new AutoShootSequence();
//=======================Test ONLY==========================
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
    private Pose flowerStart, flowerReached, flowerSlideIn, flowerBackout;
    private Pose corner1;
    private Pose turn1;
    private Pose corner2;
    private Pose turn2;
    private Pose corner3;
    private Pose turn3;
    private Pose corner4;
    private Pose turn4;
    //=======================Test ONLY==========================
    public RED_AutoGarden() {
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
        final boolean[] nearTargetDone = {false};
        final WaypointSettleGuard settle = new WaypointSettleGuard();
        final boolean useNearTargetFallback = toPosition == shootingPose || toPosition == shootingHome || toPosition == gardenPos || toPosition == gardenSlideIn || toPosition == gardenSlideOut || toPosition == flowerReached || toPosition == flowerSlideIn || toPosition == flowerBackout;
        final String movementLabel = "Waypoint to " + toPosition.x() + ", " + toPosition.y();
        return new LambdaCommand(movementLabel)
                .setStart(() -> {
                    autoStatus = movementLabel;
                    timedOut[0] = false;
                    nearTargetDone[0] = false;
                    settle.reset();
                    // Previous leg may have timed out: use measured pose instead of its target.
                    Pose actualStart = follower.pose();
                    follower.follow(line(actualStart, toPosition).linear(actualStart, toPosition));
                    movementStartedAt[0] = System.nanoTime();
                })
                .setIsDone(() -> {
                    if (!follower.isBusy()) return true;
                    // Keep correcting the exact target; only release after sustained near-target busy time.
                    if (useNearTargetFallback) {
                        Pose measured = follower.pose();
                        nearTargetDone[0] = settle.update(System.nanoTime()/1e9,
                                Math.hypot(measured.x() - toPosition.x(), measured.y() - toPosition.y()),
                                measured.heading() - toPosition.heading());
                        if (nearTargetDone[0]) return true;
                    }
                    timedOut[0] = (System.nanoTime() - movementStartedAt[0]) / 1e9
                            >= WAYPOINT_TIMEOUT_SECONDS;
                    return timedOut[0];
                })
                .setStop(interrupted -> {
                    follower.stop();
                    // Timeout advances the sequence, including any following mechanism action.
                    lastMovementResult = movementLabel + (interrupted ? ": interrupted"
                            : nearTargetDone[0] ? ": NEAR TARGET for 500 ms - advancing" : timedOut[0] ? ": TIMEOUT - advancing" : ": follower completed");
                    if (timedOut[0] && !interrupted) {
                        waypointTimeoutCount++;
                        lastWaypointTimeout = lastMovementResult;
                    }
                });
    }
    private Command buildTurnCommand(Pose targetPose) {
        return new LambdaCommand("PedroTurn")
                .setStart(() -> follower.hold(targetPose))
                .setIsDone(() -> {
                    double headingError = Math.atan2(
                            Math.sin(targetPose.heading() - follower.pose().heading()),
                            Math.cos(targetPose.heading() - follower.pose().heading())
                    );

                    return Math.abs(headingError) < Math.toRadians(3);
                })
                .setStop(interrupted -> follower.stop());
    }

    private void buildPath() {
        startPose = START.pose();
        shootingPose = GARDEN_SHOOT.pose();
        gardenApproach = GARDEN_APPROACH_START.pose();
        gardenPos = GARDEN_COLLECT_REACH.pose();
        gardenSlideIn = GARDEN_COLLECT_SLIDE_IN.pose();
        gardenSlideOut = GARDEN_COLLECT_SLIDE_OUT.pose();
        shootingHome = GARDEN_SHOOT_HOME.pose();
        flowerStart = GARDEN_FLOWER_START.pose();
        flowerReached = GARDEN_FLOWER_REACHED.pose();
        flowerSlideIn = GARDEN_FLOWER_SLIDE_IN.pose();
        flowerBackout = GARDEN_FLOWER_BACKOUT.pose();
        strafeClearancePose = GARDEN_BEFOREPARK_STRAFE.pose();
        // Zero-degree parking heading: maintain the starting orientation.
        turn1 = GARDEN_PARK.pose();
    }
    /** Require fresh RED Garden tags continuously for one second; otherwise skip shot two. */
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
                        hiveResult = "No stable Garden UP confirmation: skip shot and park";
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
    /** Runs only after the confirmed second shot finishes, with intake left running. */
    /** Bound Flower entry to one second even if Pedro remains busy. Position is not guaranteed. */
    private Command buildTimedFlowerSlideInCommand() {
        final long[] began = {0};
        return new LambdaCommand("Garden Flower timed slide-in")
                .setStart(() -> {
                    autoStatus = "Flower slide-in: timed " + GARDEN_FLOWER_SLIDE_IN_SECONDS + " s";
                    Pose actual = follower.pose();
                    follower.follow(line(actual, flowerSlideIn).linear(actual, flowerSlideIn));
                    began[0] = System.nanoTime();
                })
                .setIsDone(() -> (System.nanoTime()-began[0])/1e9 >= GARDEN_FLOWER_SLIDE_IN_SECONDS)
                .setStop(interrupted -> {
                    follower.stop();
                    lastMovementResult = "Flower slide-in: " + (interrupted ? "interrupted"
                            : "1-second timed move ended; position not verified");
                });
    }
    private Command buildFlowerCollectionAndThirdShot() {
        return new SequentialGroup(
                new InstantCommand(() -> autoStatus = "Flower collection after second shot"),
                buildMoveCommand(shootingHome, flowerStart),
                new InstantCommand(arm::liftDown),
                buildMoveCommand(flowerStart, flowerReached),
                buildTimedFlowerSlideInCommand(),
                new Delay(GARDEN_FLOWER_COLLECTION_SECONDS),
                buildMoveCommand(flowerSlideIn, flowerBackout),
                // Fold after backing out, before returning to the shooting position.
                new InstantCommand(arm::liftUp),
                buildMoveCommand(flowerBackout, shootingHome),
                new InstantCommand(() -> autoStatus = "Third shot: no additional hive check"),
                shooting.command(SHOOTER_SPEED_FRACTION, GARDEN_THIRD_SHOOT_SECONDS)
        );
    }
    private boolean allowFlowerTrip() {
        double remaining = AUTO_LIMIT_SECONDS - (System.nanoTime()-autoStartedAt)/1e9;
        boolean allowed = remaining > GARDEN_FLOWER_MIN_REMAINING_SECONDS;
        runLog.event("Flower trip decision", "remaining="+remaining+"; required > "
                +GARDEN_FLOWER_MIN_REMAINING_SECONDS+"; "+(allowed?"COLLECT":"PARK"));
        return allowed;
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
                new IfElseCommand(() -> hiveReady,
                        new SequentialGroup(
                                shooting.command(SHOOTER_SPEED_FRACTION, GARDEN_SECOND_SHOOT_SECONDS, true),
                                new IfElseCommand(() -> allowFlowerTrip(),
                                        buildFlowerCollectionAndThirdShot(),
                                        new InstantCommand(() -> autoStatus = "Insufficient time: skip Flower and park"))),
                        new InstantCommand(() -> autoStatus = "Hive not UP: skipping second shot")),
                new InstantCommand(this::prepareParking),
                buildMoveCommand(shootingHome, strafeClearancePose),
                buildMoveCommand(strafeClearancePose, turn1),
                new InstantCommand(() -> autoStatus = "Parking path complete")
        );
    }
    @Override
    public void onInit() {
        runLog = new RunTelemetryLogger();
        telemetry = runLog.attach(telemetry, getClass().getSimpleName());
        autoRunning = false;
        parkingStarted = false;
        deadlinePark = null;
        buildPath();
        shooting.init(hardwareMap); // Hardware ready but stopped during INIT.
        arm.init(hardwareMap); // Start folded; the optional flower trip lowers and then folds the arm.
        hiveTags = new HiveUpTagIDLogic(hardwareMap);
        hiveTags.start();

        // Keep createFollower() if that is the method your Constants.java currently provides.
        // The current Pedro 3 docs often name this user-defined helper create().
        follower = Constants.createFollower(hardwareMap);

        // Pedro 3 uses setPose(); setStartingPose() is no longer needed.
        //follower.setPose(startPoseStraight);
        follower.setPose(startPose);
        routine = autonomousRoutine();

    }

    @Override
    public void onUpdate() {
        runLog.updateToggle(gamepad1.back);
        runLog.gamepads(gamepad1, gamepad2);
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
        runLog.sample(hardwareMap, telemetry);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        runLog.beginRun(gamepad1.back);
        runLog.event("lifecycle", "START");
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
    
        runLog.close();
    }
}
