/* ============================================================================
 * Class: RED_AutoFlower
 * What it does:
 *   Runs the RED Flower shooting/collection route, with a 24-second parking deadline and 30-second stop.
 * Applications using it:
 *   RED_AutoFlower (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.AutoAppl;

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
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/** Flower routine in the same start-relative coordinate frame as RED_AutoGarden. */
@Autonomous(name = "RED_AutoFlower")
public class RED_AutoFlower extends NextFTCOpMode {
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
    private Follower follower;
    private HiveUpTagIDLogic hiveTags;
    private long hiveWaitStartedAt;
    private long hiveDetectedAt;
    private boolean hiveDetected;
    private boolean hiveGateDone;
    private String hiveGateResult = "Not checked";
    private Command routine, deadlinePark;
    private long autoStartedAt;
    private boolean autoRunning, parkingStarted;
    private String stage = "Ready";
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
        Path path = line(from, to).linear(from, to);
        return new LambdaCommand(label)
                .setStart(() -> { stage = label; follower.follow(path); })
                .setIsDone(() -> !follower.isBusy())
                .setStop(interrupted -> follower.stop());
    }

    /** Test-only bounded movement: timing does not guarantee reaching the five-inch target. */
    private Command buildTimedBackwardCommand() {
        Path path = line(strafeEnd, flwrPos2).linear(strafeEnd, flwrPos2);
        return new LambdaCommand("Timed backward test")
                .setStart(() -> {
                    stage = "Backward timed test: " + BACKWARD_TEST_SECONDS + " seconds";
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

    /** First shot: require >=2 Flower tags for 3 continuous seconds, or shoot on the 10-second timeout. */
    private Command buildHiveWaitCommand() {
        return new LambdaCommand("Wait for RED Flower hive")
                .setStart(() -> {
                    shooting.stop();
                    hiveTags.reset();
                    hiveWaitStartedAt = System.nanoTime();
                    hiveGateDone = false;
                    hiveDetected = false;
                    hiveGateResult = "Waiting (up to " + HIVE_WAIT_TIMEOUT_SECONDS + " seconds)";
                    stage = "FlwrPos1: checking hive tags";
                })
                .setUpdate(() -> {
                    hiveTags.update();
                    long now = System.nanoTime();
                    // Require 2, 3, or 4 distinct Flower IDs throughout the settling period.
                    // Losing the count or fresh camera data restarts the full three seconds.
                    if (hiveTags.isRedFlowerUp()) {
                        if (!hiveDetected) {
                            hiveDetected = true;
                            hiveDetectedAt = now;
                        }
                        double remaining = HIVE_SETTLE_SECONDS - (now - hiveDetectedAt) / 1e9;
                        hiveGateResult = remaining > 0
                                ? String.format("Flower count >=2; settling %.1f s", remaining)
                                : "Flower count >=2 maintained for " + HIVE_SETTLE_SECONDS + " seconds";
                        hiveGateDone = remaining <= 0;
                    } else {
                        hiveDetected = false;
                        hiveGateResult = "Flower count <2 or invalid/stale data; settling reset";
                    }
                    // Preserve the user's unconditional fallback at the overall ten-second limit.
                    if (!hiveGateDone && (now - hiveWaitStartedAt) / 1e9 >= HIVE_WAIT_TIMEOUT_SECONDS) {
                        hiveGateResult = HIVE_WAIT_TIMEOUT_SECONDS + "-second timeout: shooting WITHOUT UP confirmation";
                        hiveGateDone = true;
                    }
                })
                .setIsDone(() -> hiveGateDone)
                .setStop(interrupted -> { if (interrupted) shooting.stop(); });
    }
    /** Stop scoring before parking, including when a deadline interrupts a command. */
    private void prepareParking() {
        parkingStarted = true;
        shooting.stop();
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
            deadlinePark = buildMoveCommand(follower.pose(), finalPose,
                    PARK_DEADLINE_SECONDS + "-second deadline: direct parking from current pose");
            deadlinePark.schedule();
        }
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                buildMoveCommand(startPose, flwrPos1, "Move straight 1 inch to FlwrPos1"),
                buildHiveWaitCommand(),
                stage("FlwrPos1: shoot at " + (SHOOTER_SPEED_FRACTION * 100) + "% speed"),
                // Intake + shooter start together; feeder at 1 s, then 4 s feeding.
                shooting.command(SHOOTER_SPEED_FRACTION, FLOWER_FIRST_SHOOT_SECONDS),
                buildMoveCommand(flwrPos1, straightEnd, "Move straight 9 more inches"),
                buildMoveCommand(straightEnd, strafeEnd, "Strafe 10 inches opposite RED_AutoGarden"),
                stage("Arm down to " + IntakeArmSingleServo.DOWN_DEGREES + " degrees; wait " + FLOWER_ARM_PAUSE_SECONDS + " seconds"),
                // Use the shared down-position setting; retain the separate intermediate preset.
                new InstantCommand(() -> arm.liftDown()),
                new Delay(FLOWER_ARM_PAUSE_SECONDS),
                // To test normal completion, uncomment the next call and comment out the timed call.
                // buildMoveCommand(strafeEnd, flwrPos2, "Back 5 inches to FlwrPos2"),
                buildTimedBackwardCommand(),
                stage("FlwrPos2: wait " + FLOWER_SECOND_POSITION_PAUSE_SECONDS + " seconds"),
                new Delay(FLOWER_SECOND_POSITION_PAUSE_SECONDS),
                stage("FlwrPos2: shoot at " + (SHOOTER_SPEED_FRACTION * 100) + "% speed"),
                // Same 60% velocity target as TeleOp; 1 s startup plus 5 s feeding.
                shooting.command(SHOOTER_SPEED_FRACTION, FLOWER_SECOND_SHOOT_SECONDS),
                // Vision is no longer needed: stop polling before driving to parking.
                new InstantCommand(this::prepareParking),
                buildMoveCommand(flwrPos2, exitStraight, "Move straight 6 inches after shooting"),
                buildMoveCommand(exitStraight, finalPose, "Strafe 32 more inches"),
                stage("Complete - Limelight polling stopped")
        );
    }

    @Override public void onInit() {
        autoRunning = false;
        parkingStarted = false;
        deadlinePark = null;
        buildPath();
        shooting.init(hardwareMap);
        hiveTags = new HiveUpTagIDLogic(hardwareMap);
        hiveTags.start();
        arm.init(hardwareMap); // Fold up during INIT.
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        // Prepare commands and paths during INIT; execute only after START.
        routine = autonomousRoutine();
    }

    @Override public void onStartButtonPressed() {
        autoStartedAt = System.nanoTime();
        autoRunning = true;
        routine.schedule();
    }

    @Override public void onUpdate() {
        follower.update();
        enforceAutoDeadline();
        telemetry.addData("Auto seconds remaining", "%.1f", autoRunning
                ? Math.max(0, AUTO_LIMIT_SECONDS - (System.nanoTime() - autoStartedAt) / 1e9) : 0);
        telemetry.addData("RED_AutoFlower stage", stage);
        telemetry.addData("Shooting sequence", shooting.getStatus());
        telemetry.addData("Hive gate result", hiveGateResult);
        telemetry.addData("Hive gate observation", hiveTags.getStatus());
        telemetry.addData("RED Flower IDs (30-33)", hiveTags.getFlowerTags());
        telemetry.addData("RED Garden IDs (34-37)", hiveTags.getGardenTags());
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Intake arm target (deg)", arm.getCommandedDegrees());
        telemetry.update();
    }

    @Override public void onStop() {
        autoRunning = false;
        if (routine != null) routine.cancel();
        if (deadlinePark != null) deadlinePark.cancel();
        if (hiveTags != null) hiveTags.stop();
        shooting.stop();
        if (follower != null) follower.stop();
    }
}