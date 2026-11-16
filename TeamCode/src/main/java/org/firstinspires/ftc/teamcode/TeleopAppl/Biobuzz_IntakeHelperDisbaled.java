package org.firstinspires.ftc.teamcode.TeleopAppl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ============================================================================
 * Class: Biobuzz_IntakeHelperDisbaled
 * What it does: BioBuzzTeleop controls with the intake helper servo disabled.
 *   Legacy disabled alias; inherits the new positional shooter gate and its hardware requirement.
 * Applications using it: Separate Driver Station TeleOp entry point.
 * ============================================================================ */
@com.qualcomm.robotcore.eventloop.opmode.Disabled
@TeleOp(name = "Biobuzz_IntakeHelperDisbaled", group = "BioBuzz")
public class Biobuzz_IntakeHelperDisbaled extends BioBuzzTeleop {
    @Override protected boolean isIntakeHelperEnabled() { return false; }

    @Override public void init() {
        super.init();
        telemetry.addLine("Legacy alias: positional ShooterGateServo required; LT/RT control gate.");
    }
}
