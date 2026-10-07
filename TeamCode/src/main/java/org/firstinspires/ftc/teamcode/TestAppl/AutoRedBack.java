package org.firstinspires.ftc.teamcode.TestAppl;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static com.pedropathing.api.Paths.line;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAuto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ShooterNew;
import org.firstinspires.ftc.teamcode.subsystems.Spinner;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/** RedBack_TwoRow port from tournament-code, adapted to Pedro 3. */
@Disabled
@Autonomous(name = "AutoRedBack")
public class AutoRedBack extends NextFTCOpMode {
    private Follower follower;
    private ColorSensor colorSensor;
    private final double maxPower = 0.8;
    private final Pose startPoseStraight = new Pose(1.5, 32.4, Math.toRadians(180));
    private final Pose adjustPoseToShoot = new Pose(46, 9, Math.toRadians(140));
    private final Pose moveToPickRow = new Pose(51, 17, Math.toRadians(90));
    private final Pose moveToPick2Balls = new Pose(51, 44, Math.toRadians(90));
    private final Pose moveToPickSecondRow = new Pose(75, 17, Math.toRadians(90));
    private final Pose moveToPick2SecondBalls = new Pose(75, 52, Math.toRadians(90));
    private final Pose adjustOut = new Pose(63, 12, Math.toRadians(90));

    public AutoRedBack() {
        addComponents(new SubsystemComponent(IntakeAuto.getInstance(), Spinner.getInstance(),
                Lift.getInstance(), ShooterNew.getInstance()),
                BulkReadComponent.INSTANCE, BindingsComponent.INSTANCE);
    }

    private Command buildMoveCommand(Pose from, Pose to, double maximumPower) {
        // Pedro 2 FollowPath/PedroComponent are unavailable with Pedro 3.
        // maxPathSpeed is a speed fraction, not the old motor-power limit.
        // Preserve the existing migration cap; re-tune on the robot.
        Path path = line(from, to).linear(from, to)
                .with(Constants.foresightConfig.maxPathSpeed.at(
                        Math.min(maximumPower, Constants.foresightConfig.maxPathSpeed.get())));
        return new LambdaCommand("AutoRedBackFollowPath")
                .setStart(() -> follower.follow(path))
                .setIsDone(() -> !follower.isBusy())
                .setStop(interrupted -> follower.stop());
    }

    private Command shoot() {
        return new SequentialGroup(Lift.getInstance().LiftUpDown(),
                IntakeAuto.getInstance().startIntake, new Delay(0.5),
                Lift.getInstance().LiftUpDown(), new Delay(0.5),
                Lift.getInstance().LiftUpDown());
    }

    private Command stopAll() {
        // Run shutdown at this sequence step, not while constructing the routine.
        return new SequentialGroup(
                new InstantCommand(() -> Spinner.getInstance().setColorSensor(null)),
                Spinner.getInstance().stopSpinner(), IntakeAuto.getInstance().stopIntake,
                ShooterNew.getInstance().stopShooter());
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> Spinner.getInstance().setColorSensor(colorSensor)),
                ShooterNew.getInstance().startShooter(),
                buildMoveCommand(startPoseStraight, adjustPoseToShoot, maxPower),
                new Delay(3), shoot(),
                buildMoveCommand(adjustPoseToShoot, moveToPickRow, maxPower),
                buildMoveCommand(moveToPickRow, moveToPick2Balls, 0.35),
                IntakeAuto.getInstance().stopIntake,
                buildMoveCommand(moveToPick2Balls, adjustPoseToShoot, maxPower), shoot(),
                buildMoveCommand(adjustPoseToShoot, moveToPickSecondRow, maxPower),
                buildMoveCommand(moveToPickSecondRow, moveToPick2SecondBalls, 0.35),
                IntakeAuto.getInstance().stopIntake,
                buildMoveCommand(moveToPick2SecondBalls, moveToPickSecondRow, maxPower),
                buildMoveCommand(moveToPickSecondRow, adjustPoseToShoot, maxPower), shoot(),
                stopAll(), buildMoveCommand(adjustPoseToShoot, adjustOut, maxPower));
    }

    @Override public void onInit() {
        colorSensor = new ColorSensor(hardwareMap.get(NormalizedColorSensor.class,
                "sensor_color_distance"), telemetry);
        // Spinner.periodic() samples the sensor; no competing background reader.
        Spinner.getInstance().setColorSensor(null);
        follower = Constants.createFollower(hardwareMap);
        follower.holdEnd.set(false);
        follower.setPose(startPoseStraight);
        ShooterNew.getInstance().setShooterPowerFactor(0.65);
        Spinner.getInstance().setPower(-0.8);
    }

    @Override public void onUpdate() {
        follower.update();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override public void onStartButtonPressed() { autonomousRoutine().schedule(); }

    @Override public void onStop() {
        if (follower != null) follower.stop();
        Spinner.getInstance().setColorSensor(null);
        stopAll().schedule();
    }
}