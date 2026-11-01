/* ============================================================================
 * Class: AutoRedC1
 * What it does:
 *   Selects its named alliance/start route in HiveAutoBase; retained as a disabled autonomous draft.
 * Applications using it:
 *   AutoRedC1 (disabled).
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.AutoAppl.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/** Draft route: enable only after reviewing HiveAutoConfig and field calibration. */
@Disabled
@Autonomous(name = "AutoRedC1", group = "Hive draft")
public final class AutoRedC1 extends HiveAutoBase {
    @Override protected boolean blue() { return false; }
    @Override protected boolean north() { return false; }
}
