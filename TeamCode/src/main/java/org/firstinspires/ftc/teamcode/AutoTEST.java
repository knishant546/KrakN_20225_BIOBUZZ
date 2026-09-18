package org.firstinspires.ftc.teamcode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanism.ColorSensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ShooterNew;
//import org.firstinspires.ftc.teamcode.subsystems.Spinner;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="KrakN_Auto_0.0")
public class AutoTEST extends NextFTCOpMode {

    Pose startPoseStraight = null;
    Pose adjustPoseToShoot = null;


    Pose adjustOut = null;

    private final double maxPower = 0.8;

    private ColorSensor colorSensor ;

    public AutoTEST() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }

    private Command buildMoveCommand(Pose fromPosition,Pose toPosition, double maximumPower) {
        PathChain move = follower().pathBuilder()
                .addPath(new BezierLine(fromPosition, toPosition))
                .setLinearHeadingInterpolation(fromPosition.getHeading(), toPosition.getHeading())
                .build();
        return new FollowPath(move, false, maximumPower);
    }

    private void buildPath() {
        startPoseStraight = new Pose(1.5, 32.4, Math.toRadians(180));
        adjustPoseToShoot = new Pose(46, 9, Math.toRadians(140));

        adjustOut = new Pose(63, 12, Math.toRadians(90));
    }


    @Override
    public void onStop() {

    }

    private Command autonomousRoutine() {

        return new SequentialGroup(
                buildMoveCommand(startPoseStraight,adjustPoseToShoot,this.maxPower),
                new Delay(3),

                buildMoveCommand(adjustPoseToShoot,adjustOut,this.maxPower)
        );
    }

    private long loopTime = System.currentTimeMillis();
    @Override
    public void onInit() {
        buildPath();

        follower().setStartingPose(startPoseStraight);
        follower().setPose(startPoseStraight);

    }


    @Override
    public void onUpdate() {

        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}
