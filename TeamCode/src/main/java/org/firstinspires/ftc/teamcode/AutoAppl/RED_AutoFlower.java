/* ============================================================================
 * Class: RED_AutoFlower
 * What it does:
 *   Runs the RED Flower shooting/collection route, with a 24-second parking deadline and 30-second stop.
 * Applications using it:
 *   RED_AutoFlower (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.AutoAppl;
import org.firstinspires.ftc.teamcode.Logging.RunTelemetryLogger;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.AutoShootSequence;
import org.firstinspires.ftc.teamcode.subsystems.HiveUpTagIDLogic;
import org.firstinspires.ftc.teamcode.subsystems.IntakeArmSingleServo;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/** Flower routine in the same start-relative coordinate frame as RED_AutoGarden. */
@Autonomous(name = "RED_AutoFlower")
public class RED_AutoFlower extends NextFTCOpMode {
    private RunTelemetryLogger runLog = new RunTelemetryLogger();
    // Straight is -X as in RED_AutoGarden; opposite strafe is +Y. All headings remain zero.
    private Pose startPose;
    private Pose flwrPos1;
    private Pose straightEnd;
    private Pose strafeEnd;
    private Pose flwrPos2;
    private Pose exitStraight;
    private Pose finalPose;
    private final AutoShootSequence shooting = new AutoShootSequence();
    private final IntakeArmSingleServo arm = new IntakeArmSingleServo();
    private String lastMovementResult = "None";
    private String lastWaypointTimeout = "None";
    private int waypointTimeoutCount;
    private Follower follower;
    private HiveUpTagIDLogic hiveTags;
    private long hiveWaitStartedAt;
    private boolean hiveGateDone;
    private String hiveGateResult = "Not checked";
    private Command routine, deadlinePark;
    private long autoStartedAt;
    private boolean autoRunning, parkingStarted;
    private String stage = "Ready";
    private String flowerTripDecision = "Not checked";
    // Test only: stop the backward leg after one second, regardless of follower busy state.
    private long backwardStartedAt;

    public RED_AutoFlower() {
        addComponents(BulkReadComponent.INSTANCE, BindingsComponent.INSTANCE);
    }

    /** Define start-relative waypoints during INIT, matching RED_AutoGarden's structure. */
    private void buildPath() {
        startPose = START.pose();
        flwrPos1 = FLOWER_SHOOT_1.pose();
        straightEnd = FLOWER_STRAIGHT_END.pose();
        strafeEnd = FLOWER_STRAFE_END.pose();
        flwrPos2 = FLOWER_SHOOT_2.pose();
        exitStraight = FLOWER_EXIT_STRAIGHT.pose();
        finalPose = FLOWER_PARK.pose();
    }

    private Command buildMoveCommand(Pose from, Pose to, String label) {
        final long[] movementStartedAt = {0};
        final boolean[] timedOut = {false};
        final String movementLabel = label;
        return new LambdaCommand(movementLabel)
                .setStart(() -> {
                    shooting.closeGateKeepMotors();
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

    /** Test-only bounded movement: timing does not guarantee reaching the collection target. */
    private Command buildTimedBackwardCommand() {
        Path path = line(strafeEnd, flwrPos2).linear(strafeEnd, flwrPos2);
        return new LambdaCommand("Timed backward test")
                .setStart(() -> {
                    stage = "Backward timed test: " + BACKWARD_TEST_SECONDS + " seconds";
                    shooting.closeGateKeepMotors();
                    backwardStartedAt = System.nanoTime();
                    follower.follow(path);
                })
                .setIsDone(() -> (System.nanoTime() - backwardStartedAt) / 1e9
                        >= BACKWARD_TEST_SECONDS)
                .setStop(interrupted -> follower.stop());
    }
    private Command stage(String label) {
        return new InstantCommand(() -> stage = label);
    }

    /** First shot: require >=2 Flower tags for 3 seconds of stability (brief gaps pause), or shoot at 7 seconds. */
    private Command buildHiveWaitCommand() {
        return new LambdaCommand("Wait for RED Flower hive")
                .setStart(() -> {
                    shooting.closeGateKeepMotors();
                    hiveTags.reset();
                    hiveWaitStartedAt = System.nanoTime();
                    hiveGateDone = false;
                    hiveGateResult = "Waiting (up to " + HIVE_WAIT_TIMEOUT_SECONDS + " seconds)";
                    stage = "FlwrPos1: checking hive tags";
                })
                .setUpdate(() -> {
                    hiveTags.update();
                    long now = System.nanoTime();
                    // Any two IDs qualify; brief losses pause accumulated good-frame time.
                    hiveGateDone = hiveTags.isFlowerStable(HIVE_SETTLE_SECONDS);
                    hiveGateResult = String.format("Flower stability %.2f / %.2f s; tags %s (brief gaps pause)",
                            hiveTags.getFlowerStableSeconds(), HIVE_SETTLE_SECONDS, hiveTags.getFlowerTags());
                    // Preserve the user's unconditional fallback at the overall seven-second limit.
                    if (!hiveGateDone && (now - hiveWaitStartedAt) / 1e9 >= HIVE_WAIT_TIMEOUT_SECONDS) {
                        hiveGateResult = HIVE_WAIT_TIMEOUT_SECONDS + "-second timeout: shooting WITHOUT UP confirmation";
                        hiveGateDone = true;
                    }
                })
                .setIsDone(() -> hiveGateDone)
                .setStop(interrupted -> { if (interrupted) shooting.closeGateKeepMotors(); });
    }
    /** Stop scoring before parking, including when a deadline interrupts a command. */
    /** Same measured arrival tolerances as Garden; path timeout alone cannot allow feeding. */
    private boolean atShootingPosition() {
        Pose actual = follower.pose();
        double distance = Math.hypot(actual.x() - flwrPos1.x(), actual.y() - flwrPos1.y());
        double headingError = Math.abs(Math.atan2(Math.sin(actual.heading() - flwrPos1.heading()),
                Math.cos(actual.heading() - flwrPos1.heading())));
        return Double.isFinite(distance) && Double.isFinite(headingError)
                && distance <= GARDEN_SHOT_POSITION_TOLERANCE_INCHES
                && headingError <= Math.toRadians(GARDEN_SHOT_HEADING_TOLERANCE_DEGREES);
    }

    /** Terminal stop also applies if the final parking leg times out. */
    private void finishParking() {
        shooting.stop();
        follower.stop();
        stage = "Parking leg ended; all motors stopped (check last movement result)";
    }

    private void prepareParking() {
        parkingStarted = true;
        shooting.closeGateKeepMotors();
        arm.liftUp();
        hiveTags.stop();
        stage = "Parking";
    }

    /** Reserve the last six seconds; build the recovery path from live odometry. */
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
            stage = AUTO_LIMIT_SECONDS + " seconds: all motion stopped";
        } else if (elapsed >= PARK_DEADLINE_SECONDS && !parkingStarted) {
            if (routine != null) routine.cancel();
            follower.stop();
            prepareParking();
            // As in Garden, this direct recovery route needs field clearance validation.
                        deadlinePark = new SequentialGroup(
                    buildMoveCommand(follower.pose(), finalPose,
                            PARK_DEADLINE_SECONDS + "-second deadline: direct parking from current pose"),
                    new InstantCommand(this::finishParking));
            deadlinePark.schedule();
        }
    }

    /** Decide once immediately after shot one, using time remaining in the 30-second Auto. */
    private boolean hasTimeForFlowerTrip() {
        double remaining = AUTO_LIMIT_SECONDS - (System.nanoTime() - autoStartedAt) / 1e9;
        boolean proceed = remaining >= FLOWER_TRIP_MIN_REMAINING_SECONDS;
        flowerTripDecision = String.format("%.2f s remaining: %s (minimum %.1f s)",
                remaining, proceed ? "collect Flower" : "skip Flower and park",
                FLOWER_TRIP_MIN_REMAINING_SECONDS);
        return proceed;
    }

    private Command buildFlowerTripCommand() {
        return new SequentialGroup(
                buildMoveCommand(flwrPos1, straightEnd, "Move straight 9 more inches"),
                buildMoveCommand(straightEnd, strafeEnd, "Strafe 10 inches opposite RED_AutoGarden"),
                stage("Arm down to " + IntakeArmSingleServo.DOWN_DEGREES + " degrees; wait " + FLOWER_ARM_PAUSE_SECONDS + " seconds"),
                // Use the shared down-position setting; retain the separate intermediate preset.
                new InstantCommand(() -> arm.liftDown()),
                new Delay(FLOWER_ARM_PAUSE_SECONDS),
                // Use the timed backward workaround; do not wait for Pedro busy/completion feedback.
                buildTimedBackwardCommand(),
                // All motors keep running; the closed gate prevents shooting during collection.
                stage("Flower: collect for 2 seconds; motors running, gate CLOSED"),
                new InstantCommand(shooting::closeGateKeepMotors),
                new Delay(FLOWER_COLLECTION_SECONDS),
                // Keep all motors running during return with the gate closed.
                new InstantCommand(shooting::closeGateKeepMotors),
                buildMoveCommand(flwrPos2, strafeEnd, "Return to (-10, 10), arm down"),
                new InstantCommand(arm::liftUp),
                buildMoveCommand(strafeEnd, straightEnd, "Retrace strafe to (-10, 0), arm up"),
                buildMoveCommand(straightEnd, flwrPos1, "Return to first shooting position (-1, 0)"),
                stage("FlwrPos1: second shot; no additional hive check"),
                // Count shot duration from gate opening; recovery closes gate without extending duration.
                shooting.positionShotCommand(FLOWER_SECOND_SHOOT_SECONDS, this::atShootingPosition)
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                buildMoveCommand(startPose, flwrPos1, "Move straight 1 inch to FlwrPos1"),
                buildHiveWaitCommand(),
                stage("FlwrPos1: shoot at " + (SHOOTER_SPEED_FRACTION * 100) + "% speed"),
                // Open gate only at confirmed position after the existing hive gate permits the shot.
                shooting.positionShotCommand(FLOWER_FIRST_SHOOT_SECONDS, this::atShootingPosition),
                new IfElseCommand(this::hasTimeForFlowerTrip,
                        buildFlowerTripCommand(),
                        stage("Less than 15 seconds remain: skip Flower and park")),
                // Vision is no longer needed: stop polling before driving to parking.
                new InstantCommand(this::prepareParking),
                buildMoveCommand(flwrPos1, exitStraight, "Move straight 10 inches after shooting"),
                buildMoveCommand(exitStraight, finalPose, "Strafe 32 more inches"),
                new InstantCommand(this::finishParking)
        );
    }

    @Override public void onInit() {
        runLog = new RunTelemetryLogger();
        telemetry = runLog.attach(telemetry, getClass().getSimpleName());
        autoRunning = false;
        parkingStarted = false;
        flowerTripDecision = "Not checked";
        deadlinePark = null;
        buildPath();
        shooting.init(hardwareMap); // Includes positional gate, CLOSED during INIT.
        hiveTags = new HiveUpTagIDLogic(hardwareMap);
        hiveTags.start();
        arm.init(hardwareMap); // Fold up during INIT.
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        // Prepare commands and paths during INIT; execute only after START.
        routine = autonomousRoutine();
    }

    @Override public void onStartButtonPressed() {
        runLog.beginRun(gamepad1.back);
        runLog.event("lifecycle", "START");
        autoStartedAt = System.nanoTime();
        autoRunning = true;
        // START spins Pollen and intake; feeder waits for speed/timeout, gate waits for a shot.
        shooting.startAuto();
        routine.schedule();
    }

    @Override public void onUpdate() {
        runLog.updateToggle(gamepad1.back);
        follower.update();
        shooting.update(); // Keep startup/recovery active during travel and hive waits.
        enforceAutoDeadline();
        telemetry.addData("Auto seconds remaining", "%.1f", autoRunning
                ? Math.max(0, AUTO_LIMIT_SECONDS - (System.nanoTime() - autoStartedAt) / 1e9) : 0);
        telemetry.addData("RED_AutoFlower stage", stage);
        telemetry.addData("Shooting sequence", shooting.getStatus());
        shooting.addTelemetry(telemetry);
        telemetry.addData("Flower trip decision", flowerTripDecision);
        telemetry.addData("Hive gate result", hiveGateResult);
        telemetry.addData("Hive gate observation", hiveTags.getStatus());
        telemetry.addData("RED Flower IDs (30-33)", hiveTags.getFlowerTags());
        telemetry.addData("RED Garden IDs (34-37)", hiveTags.getGardenTags());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake arm target (deg)", arm.getCommandedDegrees());
        telemetry.addData("Last movement result", lastMovementResult);
        telemetry.addData("Waypoint timeouts", waypointTimeoutCount);
        telemetry.addData("Last waypoint timeout", lastWaypointTimeout);
        hiveTags.addDiagnostics(telemetry);
        runLog.sample(hardwareMap, telemetry);
        telemetry.update();
    }

    @Override public void onStop() {
        autoRunning = false;
        if (routine != null) routine.cancel();
        if (deadlinePark != null) deadlinePark.cancel();
        if (hiveTags != null) hiveTags.stop();
        shooting.stop();
        if (follower != null) follower.stop();
        runLog.close();
    }
}