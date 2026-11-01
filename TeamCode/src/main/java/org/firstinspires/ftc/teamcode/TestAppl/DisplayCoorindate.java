/* ============================================================================
 * Class: DisplayCoorindate
 * What it does:
 *   Displays live Pinpoint pose and captures waypoints while the robot is pushed manually.
 * Applications using it:
 *   DisplayCoorindate (Driver Station entry point).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.TestAppl;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Push-by-hand waypoint survey using the same Pinpoint configuration as autonomous.
 * No drive motor, shooter, intake, or servo is initialized or commanded.
 * Coordinates are inches in a fixed frame established at START (or X reset).
 */
@TeleOp(name = "DisplayCoorindate", group = "Test")
public class DisplayCoorindate extends OpMode {
    private PinpointLocalizer localizer;
    private boolean aWasPressed, xWasPressed;
    private final List<String> waypoints = new ArrayList<>();
    private int waypointNumber;
    private double previousHeading;
    private double accumulatedTurn;
    private String lastAction = "Keep robot still during INIT calibration";

    @Override public void init() {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        telemetry.setMsTransmissionInterval(100);
        showPose();
    }

    @Override public void init_loop() {
        localizer.update();
        showPose();
    }

    @Override public void start() {
        resetOrigin();
        aWasPressed = gamepad1.a;
        xWasPressed = gamepad1.x;
    }

    private void resetOrigin() {
        // Reset the coordinate frame only; do not recalibrate the IMU while handling the robot.
        localizer.setPose(new Pose(0, 0, 0));
        previousHeading = 0;
        accumulatedTurn = 0;
        waypoints.clear();
        waypointNumber = 0;
        lastAction = "Origin set: X=0, Y=0, heading=0";
    }

    @Override public void loop() {
        localizer.update();
        Pose pose = localizer.state().pose();
        // Unwrap heading so crossing +180/-180 does not look like a full reverse turn.
        double change = Math.atan2(Math.sin(pose.heading() - previousHeading),
                Math.cos(pose.heading() - previousHeading));
        accumulatedTurn += change;
        previousHeading = pose.heading();
        if (gamepad1.x && !xWasPressed) {
            resetOrigin();
        } else if (gamepad1.a && !aWasPressed && !gamepad1.start && !gamepad1.back) {
            waypointNumber++;
            String waypoint = String.format(Locale.US,
                    "new Pose(%.2f, %.2f, Math.toRadians(%.2f))",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            // Keep the latest eight captures readable on the Driver Station screen.
            if (waypoints.size() == 8) waypoints.remove(0);
            waypoints.add("P" + waypointNumber + ": " + waypoint);
            lastAction = "Captured P" + waypointNumber + " (screen only, not a file)";
        }
        aWasPressed = gamepad1.a;
        xWasPressed = gamepad1.x;
        showPose();
    }

    private void showPose() {
        Pose pose = localizer.state().pose();
        telemetry.addLine("Push robot by hand; no motor commands. Keep odometry pods on floor.");
        telemetry.addLine("START: zero origin | A: capture waypoint | X: zero + clear captures");
        telemetry.addData("Action", lastAction);
        telemetry.addData("Origin", "(0, 0, 0) at START / last X reset");
        telemetry.addData("Live X (in)", "%.2f", pose.x());
        telemetry.addData("Live Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Total turn since origin (deg)", "%.2f", Math.toDegrees(accumulatedTurn));
        telemetry.addLine("Heading: + counterclockwise / - clockwise (viewed from above).");
        telemetry.addData("Distance from origin (in)", "%.2f", Math.hypot(pose.x(), pose.y()));
        for (int i = 0; i < waypoints.size(); i++)
            telemetry.addData("Saved " + (i + 1), waypoints.get(i));
    }
}