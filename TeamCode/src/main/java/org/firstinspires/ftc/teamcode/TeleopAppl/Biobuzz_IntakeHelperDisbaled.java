package org.firstinspires.ftc.teamcode.TeleopAppl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* ============================================================================
 * Class: Biobuzz_IntakeHelperDisbaled
 * What it does: BioBuzzTeleop controls with the intake helper servo disabled.
 *   Does not initialize helper hardware; gamepad 2 A/B helper controls are inactive.
 * Applications using it: Separate Driver Station TeleOp entry point.
 * ============================================================================ */
@com.qualcomm.robotcore.eventloop.opmode.Disabled
@TeleOp(name = "Biobuzz_IntakeHelperDisbaled", group = "BioBuzz")
public class Biobuzz_IntakeHelperDisbaled extends BioBuzzTeleop {
    @Override protected boolean isIntakeHelperEnabled() { return false; }

    @Override public void init() {
        super.init();
        telemetry.addLine("Intake helper DISABLED: no helper hardware required; gamepad 2 A/B inactive.");
    }
}
