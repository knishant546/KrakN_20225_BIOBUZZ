package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "KrakN_Auto_0.0")
public class AutoTEST extends NextFTCOpMode {

    // Pedro 3 recommended pose creation API.
    private final PoseFactory p = PoseFactory.degrees();

    private Pose startPoseStraight;
    private Pose adjustPoseToShoot;

    private Pose moveToPickRow;
    private Pose adjustOut;

    private Follower follower;
//=======================Test ONLY==========================
    private Pose startPose;
    private Pose corner1;
    private Pose turn1;
    private Pose corner2;
    private Pose turn2;
    private Pose corner3;
    private Pose turn3;
    private Pose corner4;
    private Pose turn4;
    //=======================Test ONLY==========================
    public AutoTEST() {
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

        corner1 = new Pose(36, 0, Math.toRadians(0));
        turn1   = new Pose(36, -24, Math.toRadians(0));

        corner2 = new Pose(0, -24, Math.toRadians(0));
        turn2   = new Pose(0, 0, Math.toRadians(0));
        corner3 = new Pose(0, -12, Math.toRadians(180));

// Third right turn
        turn3 = new Pose(0, -12, Math.toRadians(90));

// Fourth side: drive forward 12" back to origin
        corner4 = new Pose(0, 0, Math.toRadians(90));

// Final right turn back to original heading
        turn4 = new Pose(0, 0, Math.toRadians(0));
        //=======================Test ONLY==========================
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
              //  buildMoveCommand(startPoseStraight, adjustPoseToShoot),
               // buildMoveCommand(adjustPoseToShoot, moveToPickRow)
                //new Delay(3),
                //buildMoveCommand(adjustPoseToShoot, moveToPickRow),
                //new Delay(3),
                //buildMoveCommand(moveToPickRow, adjustOut)
//=======================Test ONLY==========================
                // Forward 12"
                // Forward 12"
                buildMoveCommand(startPose, corner1),
                new Delay(3),
                // Turn RIGHT 90° in place
                buildMoveCommand(corner1,turn1),
                new Delay(3),
                // Forward 12"
                buildMoveCommand(turn1, corner2),
                new Delay(3),
                // Turn RIGHT 90° in place
                buildMoveCommand(corner2,turn2)
                // Side 3
               // buildMoveCommand(turn2, corner3),
                //buildTurnCommand(turn3),

                // Side 4
                //buildMoveCommand(turn3, corner4),
                //buildTurnCommand(turn4)
                //=======================Test ONLY==========================
        );
    }

    @Override
    public void onInit() {
        buildPath();

        // Keep createFollower() if that is the method your Constants.java currently provides.
        // The current Pedro 3 docs often name this user-defined helper create().
        follower = Constants.createFollower(hardwareMap);

        // Pedro 3 uses setPose(); setStartingPose() is no longer needed.
        //follower.setPose(startPoseStraight);
        follower.setPose(startPose);

    }

    @Override
    public void onUpdate() {
        // REQUIRED: Pedro 3 follower must be updated every OpMode loop.
        follower.update();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
       autonomousRoutine().schedule();
    }

    @Override
    public void onStop() {
        if (follower != null) {
            follower.stop();
        }
    }
}
