/* ============================================================================
 * Class: BLUE_AutoGarden
 * What it does:
 *   Runs the BLUE Garden shooting/collection route, with a 24-second parking deadline and 30-second stop.
 * Applications using it:
 *   BLUE_AutoGarden (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.AutoAppl;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
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

@Autonomous(name = "BLUE_AutoGarden")
public class BLUE_AutoGarden extends NextFTCOpMode {

    // Pedro 3 recommended pose creation API.
    private final PoseFactory p = PoseFactory.degrees();

    private Pose startPoseStraight;
    private Pose adjustPoseToShoot;

    private Pose moveToPickRow;
    private Pose adjustOut;

    private Follower follower;
    private Command routine, deadlinePark;
    private long autoStartedAt;
    private boolean autoRunning, parkingStarted;
    private static final double PARK_DEADLINE_SECONDS = 24.0;
    private static final double AUTO_LIMIT_SECONDS = 30.0;
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
    private boolean hiveReady;
    private String hiveResult = "First shot: no check required";
    private Pose strafeClearancePose;
    private Pose corner1;
    private Pose turn1;
    private Pose corner2;
    private Pose turn2;
    private Pose corner3;
    private Pose turn3;
    private Pose corner4;
    private Pose turn4;
    //=======================Test ONLY==========================
    public BLUE_AutoGarden() {
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
        Path move = line(fromPosition, toPosition)
                .linear(fromPosition, toPosition);

        return new LambdaCommand("PedroFollowPath")
                .setStart(() -> follower.follow(move))
                .setIsDone(() -> !follower.isBusy())
                .setStop(interrupted -> follower.stop());
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

    // BLUE placement is the RED setup rotated 180 degrees on the field.
    // Resetting that starting pose to (0,0,0) preserves these local coordinates/headings.
    private void buildPath() {
        // Same coordinates/headings as before, expressed with the Pedro 3 PoseFactory.
       // startPoseStraight = new Pose(0, 0, Math.toRadians(0));
       // adjustPoseToShoot = new Pose(12, -18, Math.toRadians(180));
      //  moveToPickRow = new Pose(0, 0., Math.toRadians(180));

        //adjustPoseToShoot = new Pose(11, 4, Math.toRadians(24));

       // moveToPickRow = new Pose(27, 18.6, Math.toRadians(90));
        //adjustOut = startPoseStraight;
//=======================Test ONLY==========================
        startPose = new Pose(0, 0, Math.toRadians(0));
        // Same straight direction as BLUE_AutoFlower: move one inch in -X before shooting.
        shootingPose = new Pose(-1, 0, Math.toRadians(0));
        gardenApproach = new Pose(-4, 0, 0);
        gardenPos = new Pose(-4, -52.89, 0);

        // Preserve the current 3-inch sideways clearance from the new shooting pose.
        strafeClearancePose = new Pose(-1, -3, Math.toRadians(0));
        // Measured with DisplayCoorindate from the same start-relative origin.
        // Heading 268.76 degrees is equivalent to -91.24 degrees (clockwise).
        turn1 = new Pose(-73.16, -45.13, Math.toRadians(-91.24));
/*
        corner2 = new Pose(0, -24, Math.toRadians(0));
        turn2   = new Pose(0, 0, Math.toRadians(0));
        corner3 = new Pose(0, -12, Math.toRadians(180));

// Third right turn
        turn3 = new Pose(0, -12, Math.toRadians(90));

// Fourth side: drive forward 12" back to origin
        corner4 = new Pose(0, 0, Math.toRadians(90));

// Final right turn back to original heading
        turn4 = new Pose(0, 0, Math.toRadians(0));*/
        //=======================Test ONLY==========================
    }

    /** Second shot only: Garden IDs 42..45 indicate UP by the camera-mount heuristic. */
    private Command buildGardenHiveWaitCommand() {
        return new LambdaCommand("Check Garden hive")
                .setStart(() -> {
                    shooting.stop();
                    hiveTags.reset();
                    hiveWaitStarted = System.nanoTime();
                    hiveReady = false;
                    hiveResult = "Waiting for >=2 Garden tags, up to 2 seconds";
                })
                .setUpdate(() -> {
                    hiveTags.update();
                    if (hiveTags.isBlueGardenUp()) {
                        hiveReady = true;
                        hiveResult = "Garden tags detected: proceed to second shot";
                    } else if ((System.nanoTime() - hiveWaitStarted) / 1e9 >= 2.0) {
                        hiveReady = true;
                        hiveResult = "2-second timeout: shoot without UP confirmation";
                    }
                })
                .setIsDone(() -> hiveReady);
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
            autoStatus = "30 seconds: all motion stopped";
        } else if (elapsed >= PARK_DEADLINE_SECONDS && !parkingStarted) {
            if (routine != null) routine.cancel();
            follower.stop();
            prepareParking();
            autoStatus = "24-second deadline: direct parking from current pose";
            // This direct recovery path needs field clearance validation from collection locations.
            deadlinePark = buildMoveCommand(follower.pose(), turn1);
            deadlinePark.schedule();
        }
    }
    private Command autonomousRoutine() {
        return new SequentialGroup(
                buildMoveCommand(startPose, shootingPose),
                shooting.command(AutoShootSequence.SHOOTER_SPEED_FRACTION, 6.0),
                // Collect at gardenPos, then retrace the separate straight/strafe legs.
                buildMoveCommand(shootingPose, gardenApproach),
                buildMoveCommand(gardenApproach, gardenPos),
                new Delay(0.5),
                shooting.collectCommand(2.0),
                buildMoveCommand(gardenPos, gardenApproach),
                buildMoveCommand(gardenApproach, shootingPose),
                buildGardenHiveWaitCommand(),
                shooting.command(AutoShootSequence.SHOOTER_SPEED_FRACTION, 4.0),
                // Vision is no longer needed: stop polling before driving to parking.
                new InstantCommand(this::prepareParking),
                buildMoveCommand(shootingPose, strafeClearancePose),
                buildMoveCommand(strafeClearancePose, turn1), new InstantCommand(() -> autoStatus = "Parking path complete")
        );
    }
    @Override
    public void onInit() {
        autoRunning = false;
        parkingStarted = false;
        deadlinePark = null;
        buildPath();
        shooting.init(hardwareMap); // Hardware ready but stopped during INIT.
        arm.init(hardwareMap); // Keep the intake arm folded up during this two-shot routine.
        hiveTags = new HiveUpTagIDLogic(hardwareMap, HiveUpTagIDLogic.Alliance.BLUE);
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
