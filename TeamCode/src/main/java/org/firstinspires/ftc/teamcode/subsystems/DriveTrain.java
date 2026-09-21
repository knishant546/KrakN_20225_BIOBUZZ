package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class DriveTrain implements Subsystem {
    private static final DriveTrain INSTANCE = new DriveTrain();

    public static DriveTrain getInstance() { return INSTANCE; }
    private DriveTrain(){}

    @Override
    public void initialize() {
        frontLeftMotor.setPower(0.8);
        frontRightMotor.setPower(0.8);
        backLeftMotor.setPower(0.8);
        backRightMotor.setPower(0.8);
    }

    private final MotorEx frontLeftMotor = new MotorEx("FLDC");
    private final MotorEx frontRightMotor = new MotorEx("FRDC").reversed();
    private final MotorEx backLeftMotor = new MotorEx("RLDC");
    private final MotorEx backRightMotor = new MotorEx("RRDC").reversed();

    public DriverControlledCommand startDrive = new MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX(),
            Gamepads.gamepad1().rightStickX()
    );

}
