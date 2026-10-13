package org.firstinspires.ftc.teamcode.AutoAppl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/** Draft route: enable only after reviewing HiveAutoConfig and field calibration. */
@Disabled
@Autonomous(name = "AutoRedC1", group = "Hive draft")
public final class AutoRedC1 extends HiveAutoBase {
    @Override protected boolean blue() { return false; }
    @Override protected boolean north() { return false; }
}
