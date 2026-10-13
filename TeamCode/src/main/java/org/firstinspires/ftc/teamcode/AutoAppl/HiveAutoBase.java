package org.firstinspires.ftc.teamcode.AutoAppl;

import static com.pedropathing.api.Paths.line;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.limelightvision.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.subsystems.ShooterNew;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import java.util.*;

/**
 * Draft BIOBUZZ routes. Origin=audience/red corner; +X=blue, +Y=scoring side,
 * inches, heading CCW from +X. Verify this frame against Pinpoint on the robot.
 * All driving offsets below are placeholders; none were measured on your robot.
 * FTC cluster guidance: https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html
 * Limelight individual-tag Euler angles are NOT FTC cluster roll. Use calibrated
 * per-tag references at the fixed shooting pose, never blindly apply +/-90 degrees.
 */
public abstract class HiveAutoBase extends OpMode {
    protected abstract boolean blue();
    protected abstract boolean north();
    private enum State { MOVE, SHOOT, COLLECT, PARK, DONE }
    private static class Stop {
        final Pose pose,sweepEnd; final boolean shoot;
        Stop(Pose p,boolean s) { this(p,s,null); }
        Stop(Pose p,boolean s,Pose end) { pose=p; shoot=s; sweepEnd=end; }
    }
    private final List<Stop> stops=new ArrayList<>();
    private final ArrayDeque<Pose> waypoints=new ArrayDeque<>();
    private final HiveAutoLogic logic=new HiveAutoLogic();
    private Follower follower;
    private Limelight3A camera;
    private ShooterNew shooter;
    private DcMotor intake,spinner;
    private Servo left,right;
    private NormalizedColorSensor pollenSensor;
    private final YellowPollenCounter intakeCounter=new YellowPollenCounter();
    private State state=State.DONE;
    private Pose position,target;
    private double stateAt,moveAt,startAt,simFrame,frame=Double.NaN;
    private boolean initialized,dry,turning;
    private int stopIndex,inventory=HiveAutoConfig.POLLEN_CAPACITY,totalAttempts,attemptBase,tagFirst;
    private String status="Not initialized";
    private double angle=Double.NaN;

    private static Pose p(double x,double y,double deg) { return new Pose(x,y,Math.toRadians(deg)); }
    private double lane() { return blue()?108:36; }
    private Pose shooting() { return p(blue()?84:60,north()?132:12,north()?-90:90); }
    private Pose parking() { return p(blue()?135:9,blue()?36:108,blue()?0:180); }

    @Override public void init() {
        dry=HiveAutoConfig.DRY_RUN;
        // Wall start outside loading zones. 9-inch offset assumes an 18-inch robot.
        position=p(blue()?84:60,north()?135:9,north()?-90:90);
        tagFirst=blue()?(north()?42:38):(north()?30:34);
        stops.add(new Stop(shooting(),true));
        Pose flower=blue()?(north()?p(132,96,0):p(96,12,-90))
                          :(north()?p(48,132,90):p(12,48,180));
        stops.add(new Stop(flower,false)); stops.add(new Stop(shooting(),true));
        if(blue()==north()) { // Blue D6 and Red C1 include garden pickup and another shoot.
            // Garden pollen forms a row: approach from the inner end, then sweep outward.
            stops.add(new Stop(blue()?p(108,132,0):p(36,12,180),false,
                    blue()?p(132,132,0):p(12,12,180)));
            stops.add(new Stop(shooting(),true));
        }
        if(!dry && (!HiveAutoConfig.ROUTES_CALIBRATED || !HiveAutoConfig.TAG_POSES_CALIBRATED
                || !HiveAutoConfig.FEED_COUNTERS_CALIBRATED)) {
            status="LIVE BLOCKED: calibrate routes, vision, feeder and color counter"; return;
        }
        if(!dry) {
            follower=Constants.createFollower(hardwareMap); follower.holdEnd.set(false); follower.setPose(position);
            shooter=ShooterNew.getInstance(); shooter.initialize(hardwareMap); shooter.stopImmediately();
            intake=hardwareMap.get(DcMotor.class,"intake"); spinner=hardwareMap.get(DcMotor.class,"spinner");
            intake.setPower(0); spinner.setPower(0);
            left=hardwareMap.get(Servo.class,"leftServo"); right=hardwareMap.get(Servo.class,"rightServo");
            left.setDirection(Servo.Direction.REVERSE); right.setDirection(Servo.Direction.FORWARD);
            pollenSensor=hardwareMap.get(NormalizedColorSensor.class,HiveAutoConfig.COLOR_SENSOR);
            pollenSensor.setGain(HiveAutoConfig.COLOR_GAIN);
            camera=hardwareMap.get(Limelight3A.class,"limelight"); camera.setPollRateHz(50);
            camera.pipelineSwitch(HiveAutoConfig.PIPELINE);
            // Do not move servos during INIT; set the feeder down physically before START.
        }
        initialized=true; status=dry?"DRY RUN: no hardware mapped":"Ready";
        telemetry.setMsTransmissionInterval(150); report();
    }
    @Override public void init_loop() { report(); }
    @Override public void start() {
        if(!initialized) { requestOpModeStop(); return; }
        startAt=getRuntime(); stopIndex=0;
        if(!dry) {
            setFeed(false); camera.start();
        }
        navigate(stops.get(0).pose,false,0);
    }
    private double now() { return getRuntime()-startAt; }
    private void navigate(Pose goal,boolean park,double t) {
        stopMechanisms(); state=park?State.PARK:State.MOVE; stateAt=t; waypoints.clear();
        // Route through the outer alliance corridor instead of diagonally through the hive.
        Pose at=dry?position:follower.pose();
        if(Math.hypot(at.x()-goal.x(),at.y()-goal.y())>6) {
            waypoints.add(p(lane(),at.y(),Math.toDegrees(at.heading())));
            waypoints.add(p(lane(),goal.y(),Math.toDegrees(goal.heading())));
        }
        waypoints.add(goal); nextLeg(t);
    }
    private void nextLeg(double t) {
        target=waypoints.poll(); moveAt=t;
        if(target!=null && !dry) {
            Pose at=follower.pose();
            turning=Math.hypot(at.x()-target.x(),at.y()-target.y())<0.1;
            // Pedro lines require distinct endpoints; hold handles an in-place heading change.
            if(turning)follower.hold(target);
            else follower.follow(line(at,target).linear(at,target)
                    .with(Constants.foresightConfig.maxPathSpeed.at(HiveAutoConfig.PATH_SPEED)));
        }
    }
    private void park(double t,String why) { status=why; navigate(parking(),true,t); }
    private void nextStop(double t) {
        if(++stopIndex>=stops.size())park(t,"Route complete: park");
        else navigate(stops.get(stopIndex).pose,false,t);
    }
    @Override public void loop() {
        if(!initialized) { report(); return; }
        double t=now();
        if(t>=HiveAutoConfig.STOP_AT) { stopAll(); state=State.DONE; status="AUTO deadline"; report(); return; }
        if(state==State.DONE) { report(); return; }
        // Parking overrides shooting, collection, and travel, even midway through a feed.
        if(t>=HiveAutoConfig.PARK_AT && state!=State.PARK)park(t,"Time reserved for parking");
        if(!dry)follower.update();
        boolean entry=false;
        if(!dry && state==State.COLLECT) {
            // One coherent sample per loop; count only during active pickup.
            NormalizedRGBA c=pollenSensor.getNormalizedColors();
            entry=intakeCounter.update(t,c.red,c.green,c.blue,c.alpha);
        }
        switch(state) {
            case MOVE: case PARK:
                boolean arrived=dry?t-moveAt>=0.18:turning?
                        Math.abs(wrap(Math.toDegrees(follower.pose().heading()-target.heading())))
                            <=HiveAutoConfig.HEADING_TOLERANCE:!follower.isBusy();
                if(t-moveAt>HiveAutoConfig.MOVE_TIMEOUT) {
                    stopAll(); state=State.DONE; status="Path timeout: parking not guaranteed"; break;
                }
                if(!arrived)break;
                if(!dry && (Math.hypot(follower.pose().x()-target.x(),follower.pose().y()-target.y())
                        >HiveAutoConfig.POSITION_TOLERANCE || Math.abs(wrap(Math.toDegrees(
                        follower.pose().heading()-target.heading())))>HiveAutoConfig.HEADING_TOLERANCE)) {
                    stopAll(); state=State.DONE; status="Path endpoint not reached"; break;
                }
                position=target;
                if(!waypoints.isEmpty()) { nextLeg(t); break; }
                if(state==State.PARK) { stopAll(); state=State.DONE; status="Parked"; break; }
                stateAt=t;
                if(stops.get(stopIndex).shoot) {
                    state=State.SHOOT; logic.begin(t,inventory); attemptBase=totalAttempts;
                    if(!dry) { follower.stop(); if(inventory>0)shooter.runVelocity(HiveAutoConfig.VELOCITY); }
                } else {
                    state=State.COLLECT; if(!dry) {
                        follower.stop();
                        Pose end=stops.get(stopIndex).sweepEnd;
                        if(end!=null && inventory<HiveAutoConfig.POLLEN_CAPACITY)follower.follow(line(follower.pose(),end)
                                .linear(follower.pose(),end)
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.15)));
                    }
                    setCollect(inventory<HiveAutoConfig.POLLEN_CAPACITY);
                }
                break;
            case SHOOT:
                HiveAutoLogic.Hive hive=readHive();
                boolean ready=dry || Math.abs(shooter.measuredVelocity()-HiveAutoConfig.VELOCITY)
                        <=HiveAutoConfig.VELOCITY_TOLERANCE;
                logic.tick(t,hive,frame,ready);
                inventory=logic.balls; totalAttempts=attemptBase+logic.attempts;
                setFeed(logic.feed); status=logic.reason;
                if(logic.phase==HiveAutoLogic.Phase.FAULT)park(t,logic.reason);
                else if(logic.phase==HiveAutoLogic.Phase.DONE)nextStop(t);
                break;
            case COLLECT:
                if(entry)inventory++;
                // Stop both intake mechanisms on the very loop that reaches four.
                if(inventory>=HiveAutoConfig.POLLEN_CAPACITY)setCollect(false);
                if(inventory>HiveAutoConfig.POLLEN_CAPACITY) { stopAll(); state=State.DONE; status="Inventory overflow"; break; }
                if(dry && t-stateAt>=0.3)inventory=HiveAutoConfig.POLLEN_CAPACITY; // Simulation only; never assume hardware pickup.
                if(inventory>=HiveAutoConfig.POLLEN_CAPACITY || t-stateAt>=HiveAutoConfig.COLLECT_TIMEOUT) {
                    if(!dry)follower.stop();
                    setCollect(false); nextStop(t);
                }
                break;
            default: break;
        }
        report();
    }
    private HiveAutoLogic.Hive readHive() {
        frame=Double.NaN; angle=Double.NaN;
        if(dry) { frame=++simFrame; return HiveAutoConfig.SIM_INITIAL_UP && totalAttempts<HiveAutoConfig.SIM_TIP_AFTER
                ?HiveAutoLogic.Hive.UP:HiveAutoLogic.Hive.DOWN; }
        LLResult r=camera.getLatestResult();
        if(!camera.isConnected() || !camera.isRunning() || r==null || !r.isValid()
                || r.getStaleness()>200 || r.getPipelineIndex()!=HiveAutoConfig.PIPELINE)
            return HiveAutoLogic.Hive.UNKNOWN;
        frame=r.getTimestamp();
        if(r.getFiducialResults()==null)return HiveAutoLogic.Hive.UNKNOWN;
        HiveAutoLogic.Hive result=null;
        for(LLResultTypes.FiducialResult tag:r.getFiducialResults()) {
            int id=tag.getFiducialId(); if(id<tagFirst || id>tagFirst+3)continue;
            Pose3D pose=tag.getTargetPoseCameraSpace();
            if(pose==null || pose.getOrientation()==null || pose.getPosition()==null)return HiveAutoLogic.Hive.UNKNOWN;
            Position p=pose.getPosition().toUnit(DistanceUnit.METER);
            double range=Math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
            YawPitchRollAngles a=pose.getOrientation();
            angle=HiveAutoConfig.POSE_AXIS==0?a.getRoll(AngleUnit.DEGREES):
                    HiveAutoConfig.POSE_AXIS==1?a.getPitch(AngleUnit.DEGREES):a.getYaw(AngleUnit.DEGREES);
            double tx=tag.getTargetXDegrees();
            if(!Double.isFinite(range)||range<HiveAutoConfig.MIN_RANGE_M||range>HiveAutoConfig.MAX_RANGE_M
                    ||!Double.isFinite(angle)||!Double.isFinite(tx))return HiveAutoLogic.Hive.UNKNOWN;
            double error=Math.abs(wrap(angle-HiveAutoConfig.UPRIGHT_DEG[id-30]));
            HiveAutoLogic.Hive h=error<=HiveAutoConfig.UP_TOLERANCE?HiveAutoLogic.Hive.UP:
                    error>=HiveAutoConfig.DOWN_MIN?HiveAutoLogic.Hive.DOWN:HiveAutoLogic.Hive.UNKNOWN;
            if(h==HiveAutoLogic.Hive.UP && Math.abs(tx-HiveAutoConfig.AIM_TX_DEG[id-30])>
                    HiveAutoConfig.AIM_TOLERANCE)h=HiveAutoLogic.Hive.UNKNOWN;
            if(result!=null && result!=h)return HiveAutoLogic.Hive.UNKNOWN;
            result=h;
        }
        return result==null?HiveAutoLogic.Hive.UNKNOWN:result;
    }
    private static double wrap(double a) { return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(a)),Math.cos(Math.toRadians(a)))); }
    private void setFeed(boolean up) {
        if(!dry && left!=null && right!=null) {
            left.setPosition(up?HiveAutoConfig.LIFT_UP:HiveAutoConfig.LIFT_DOWN);
            right.setPosition(up?HiveAutoConfig.LIFT_UP:HiveAutoConfig.LIFT_DOWN);
        }
    }
    private void setCollect(boolean on) {
        if(!dry && intake!=null && spinner!=null) { intake.setPower(on?-1:0); spinner.setPower(on?-0.65:0); }
    }
    private void stopMechanisms() { if(shooter!=null)shooter.stopImmediately(); setCollect(false); if(initialized)setFeed(false); }
    private void stopAll() { stopMechanisms(); if(follower!=null)follower.stop(); }
    @Override public void stop() { stopAll(); if(camera!=null)camera.stop(); }
    private void report() {
        telemetry.addData("Route",(blue()?"Blue D":"Red C")+(north()?"6":"1"));
        telemetry.addData("Mode",dry?"DRY RUN - NO HARDWARE":"LIVE");
        telemetry.addData("State",state); telemetry.addData("Status",status);
        telemetry.addData("Target tags",tagFirst+".."+(tagFirst+3));
        telemetry.addData("Estimated load / feed attempts",inventory+" / "+totalAttempts);
        telemetry.addData("Shot phase",logic.phase); telemetry.addData("Tag angle",angle);
        telemetry.addData("Park at / stop at",HiveAutoConfig.PARK_AT+" / "+HiveAutoConfig.STOP_AT);
        telemetry.addData("Elapsed",initialized?now():0); telemetry.update();
    }
}
