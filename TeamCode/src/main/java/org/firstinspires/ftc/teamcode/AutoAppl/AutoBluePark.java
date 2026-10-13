package org.firstinspires.ftc.teamcode.AutoAppl;

import static com.pedropathing.api.Paths.line;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;

/** Parking-only test. Field inches: +X toward F, +Y toward row 6.
 * Place chassis CENTER at D6 (84,132), facing row 5 (-90 degrees).
 * Uses AutoTEST's command pattern and shared drivetrain/Pinpoint settings.
 */
@Autonomous(name="Blue Park - D6 to F2", group="Chassis test")
public final class AutoBluePark extends NextFTCOpMode {
    private static final double AUTO_TIMEOUT=30, BETWEEN_LEGS=3;
    private Follower follower;
    private Pose startPose,d5,e5,e2,f2,target;
    private Command routine;
    private boolean started,done;
    private int leg;
    private double startTime,legTime;
    private String status="Ready at D6, face row 5; press START";

    public AutoBluePark() {
        addComponents(BulkReadComponent.INSTANCE,BindingsComponent.INSTANCE);
    }
    private void buildPath() {
        double h=Math.toRadians(-90);
        startPose=new Pose(84,132,h);
        d5=new Pose(84,108,h);
        e5=new Pose(108,108,h);
        e2=new Pose(108,36,h);
        f2=new Pose(132,36,h);
    }
    private Command buildMoveCommand(Pose from,Pose to,int number) {
        // Construct during INIT; use the same speed and holdEnd default as AutoTEST.
        Path path=line(from,to).linear(from,to);
        return new LambdaCommand("BlueParkLeg"+number)
                .setStart(() -> {
                    target=to; leg=number; legTime=getRuntime();
                    status="Driving leg "+number+" / 4";
                    follower.follow(path);
                })
                .setIsDone(() -> !follower.isBusy())
                .setStop(interrupted -> {
                    follower.stop();
                    if(!interrupted)status="Leg "+number+" complete; pause";
                });
    }
    private Command autonomousRoutine() {
        return new SequentialGroup(
                buildMoveCommand(startPose,d5,1),new Delay(BETWEEN_LEGS),
                buildMoveCommand(d5,e5,2),new Delay(BETWEEN_LEGS),
                buildMoveCommand(e5,e2,3),new Delay(BETWEEN_LEGS),
                buildMoveCommand(e2,f2,4),
                new LambdaCommand("BlueParkComplete")
                        .setStart(() -> {done=true; status="Route complete: stopped"; follower.stop();})
                        .setIsDone(() -> true));
    }
    @Override public void onInit() {
        buildPath();
        follower=Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        routine=autonomousRoutine();
    }
    @Override public void onStartButtonPressed() {
        started=true; startTime=getRuntime();
        routine.schedule();
    }
    @Override public void onUpdate() {
        // Match AutoTEST: update Pedro every loop, including INIT and command pauses.
        if(started && !done && getRuntime()-startTime>=AUTO_TIMEOUT) {
            done=true;
            routine.cancel(); // Cancel the group so a later leg cannot restart the motors.
            follower.stop();
            status="30 second deadline: stopped";
        }
        follower.update();
        telemetry.addData("Start","D6, face row 5");
        telemetry.addData("Status",status);
        telemetry.addData("Field position",follower.pose());
        telemetry.addData("Displacement from start (in)","X %.2f, Y %.2f",
                follower.pose().x()-startPose.x(),follower.pose().y()-startPose.y());
        telemetry.addData("Follower mode",follower.mode());
        telemetry.addData("Busy / parametric end",follower.isBusy()+" / "+follower.atParametricEnd());
        if(target!=null) {
            telemetry.addData("Target",target);
            telemetry.addData("Position error (in)",Math.hypot(target.x()-follower.pose().x(),target.y()-follower.pose().y()));
            double h=target.heading()-follower.pose().heading();
            telemetry.addData("Heading error (deg)",Math.toDegrees(Math.atan2(Math.sin(h),Math.cos(h))));
            telemetry.addData("Leg / seconds since leg start",leg+" / "+(getRuntime()-legTime));
        }
        telemetry.addData("Auto seconds",started?getRuntime()-startTime:0);
        telemetry.update();
    }
    @Override public void onStop() {
        if(routine!=null)routine.cancel();
        if(follower!=null)follower.stop();
    }
}
