package org.firstinspires.ftc.teamcode.AutoAppl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/** Draft route: enable only after reviewing HiveAutoConfig and field calibration. */
@Disabled
@Autonomous(name = "AutoBlueD1", group = "Hive draft")
public final class AutoBlueD1 extends HiveAutoBase {
    @Override protected boolean blue() { return true; }
    @Override protected boolean north() { return false; }
}
