package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake  {

    private DcMotor intakeMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake");
    }

    public void  start() {
        intakeMotor.setPower(-1.0);

    }

    public void stop() {
        intakeMotor.setPower(0.0);

    }
}
