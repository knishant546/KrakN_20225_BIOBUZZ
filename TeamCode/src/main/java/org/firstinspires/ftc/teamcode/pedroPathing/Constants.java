package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    /*
     * ============================================================
     * DRIVETRAIN - Pedro 3 API++++++++
     * ============================================================
     *
     * These names/directions were carried over from your Pedro 2
     * MecanumConstants.
     */
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FLDC");
                c.backLeftName.set("RLDC");
                c.frontRightName.set("FRDC");
                c.backRightName.set("RRDC");



                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);


                c.manualBrakeMode.set(true);
            }
    );


    /*
     * ============================================================
     * PINPOINT LOCALIZER - Pedro 3 API
     * ============================================================
     *
     * Pedro 3 names the forward Pinpoint pod the X pod and the
     * strafe Pinpoint pod the Y pod.
     *
     * Migrated from your old values:
     *   forwardPodY  =  0.3
     *   strafePodX  = -5.5
     *
     * IMPORTANT:
     * Re-run the Pedro 3 Pinpoint AutoTune when possible and replace
     * the offsets/directions with the generated values.
     */
    public static final PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");

                c.xPodOffset.set(0.1);
                c.yPodOffset.set(6.25);

                c.offsetUnits.set(DistanceUnit.INCH);
                c.globalDistanceUnit.set(DistanceUnit.INCH);

                c.podType.set(
                        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
                );

                c.xPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.REVERSED
                );
                c.yPodDirection.set(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                );
            }
    );


    /*
     * ============================================================
     * FORESIGHT - Pedro 3 path-following algorithm
     * ============================================================
     *
     * Pedro 3 replaced:
     *   FollowerConstants
     *   PathConstraints
     *   MecanumConstants.xVelocity/yVelocity
     *
     * with ForesightConfig.
     *
     * The values below are STARTING/PLACEHOLDER values based on the
     * current Pedro 3 example configuration. They are NOT your robot's
     * tuned values.
     *
     * BEFORE running autonomous at competition speed:
     *   1. Run Pedro 3 Mecanum AutoTune
     *   2. Run Pinpoint AutoTune
     *   3. Run Foresight AutoTune
     *   4. Replace this whole ForesightConfig with the generated one
     *
     * maxPathSpeed is temporarily limited to 0.25 while migrating.
     */
    public static final ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryForward =
                        Controller.proportional(0.3);
                Controller secondaryForward =
                        Controller.proportional(0.1);

                Controller primaryStrafe =
                        Controller.proportional(0.3);
                Controller secondaryStrafe =
                        Controller.proportional(0.1);

                c.forwardTranslational.set(
                        Controller.piecewise(secondaryForward)
                                .put(2.5, primaryForward)
                );

                c.strafeTranslational.set(
                        Controller.piecewise(secondaryStrafe)
                                .put(2.5, primaryStrafe)
                );

                c.coast.set(
                        Controller.proportionalFeedforward(
                                0.010978350889324107
                        )
                );

                c.brake.set(
                        Controller.proportionalFeedforward(
                                0.008731598255925491
                        )
                );

                c.headingFeedback.set(
                        Controller.proportional(
                                5.258721785960744
                        )
                );

                c.headingBrakeCoefficients.set(
                        Vector2D.cartesian(
                                0.05642143125655298,
                                0.0063829525363003695
                        )
                );

                c.linearBrakeCoefficients.set(
                        Matrix.diag(
                                0.10605894992901523,
                                0.08719146175596092
                        )
                );

                c.quadraticBrakeCoefficients.set(
                        Matrix.diag(
                                0.0014663966976606565,
                                0.0013837064502458813
                        )
                );

                c.maxAchievableForwardVelocity.set(
                        72.72923108818539
                );

                c.maxAchievableStrafeVelocity.set(
                        52.34323936525474
                );

                c.naturalForwardDeceleration.set(
                        85.01144677379789
                );

                c.naturalStrafeDeceleration.set(
                        104.49787535782846
                );

                // Temporary migration speed limit.
                // Remove/change after AutoTune.
                c.maxPathSpeed.set(0.60);
            }
    );


    /*
     * Current Pedro 3 documentation uses Constants.create(hardwareMap).
     */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }


    /*
     * Compatibility alias so your already-migrated AutoTEST.java can
     * continue calling Constants.createFollower(hardwareMap).
     *
     * New code can use create().
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return create(hardwareMap);
    }
}
