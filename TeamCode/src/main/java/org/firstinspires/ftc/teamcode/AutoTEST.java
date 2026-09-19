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
    private Pose adjustOut;

    private Follower follower;

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
                .setIsDone(() -> !follower.following())
                .setStop(interrupted -> follower.stop());
    }

    private void buildPath() {
        // Same coordinates/headings as before, expressed with the Pedro 3 PoseFactory.
        startPoseStraight = p.of(1.5, 32.4, 180);
        adjustPoseToShoot = p.of(46, 9, 140);
        adjustOut = p.of(63, 12, 90);
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                buildMoveCommand(startPoseStraight, adjustPoseToShoot),
                new Delay(3),
                buildMoveCommand(adjustPoseToShoot, adjustOut)
        );
    }

    @Override
    public void onInit() {
        buildPath();

        // Keep createFollower() if that is the method your Constants.java currently provides.
        // The current Pedro 3 docs often name this user-defined helper create().
        follower = Constants.createFollower(hardwareMap);

        // Pedro 3 uses setPose(); setStartingPose() is no longer needed.
        follower.setPose(startPoseStraight);
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
