/* ============================================================================
 * Class: ColorSensor
 * What it does:
 *   Reads normalized color telemetry and detects object presence using an alpha threshold, not yellow classification.
 * Applications using it:
 *   AutoBlueD1 [disabled draft]
 *   AutoBlueD6 [disabled draft]
 *   AutoRedBack [excluded from build]
 *   AutoRedC1 [disabled draft]
 *   AutoRedC6 [disabled draft]
 *   DecodeTeleop [excluded from build]
 *   KKN_Limelight
 *   KKN_TeleOP
 *   Includes indirect/source dependencies; optional features may be inactive.
 * ============================================================================ */
package org.firstinspires.ftc.teamcode.subsystems.test;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;


public class ColorSensor implements Runnable{

    private final NormalizedColorSensor colorSensor;
    private final Telemetry telemetry;
    private boolean isDetected = false;

    private boolean shouldRun = false;

    public ColorSensor(
            NormalizedColorSensor colorSensor,
            Telemetry telemetry) {

        this.colorSensor = colorSensor;
        this.telemetry = telemetry;

        init();
    }

    public void init() {
        this.isDetected = false;
        colorSensor.setGain(20);
    }

    public void startColorSensor(){
        this.shouldRun = true;
    }

    public void stopColorSensor(){
        this.shouldRun = false;
    }

    public boolean isDetected(){
        return this.isDetected;
    }

    public boolean isObjectDetected() {
        float alphaValue = colorSensor.getNormalizedColors().alpha;
        telemetry.addData(
                "Red value",
                colorSensor.getNormalizedColors().red);

        telemetry.addData(
                "Blue value",
                colorSensor.getNormalizedColors().blue);

        telemetry.addData(
                "Green value",
                colorSensor.getNormalizedColors().green);

        telemetry.addData(
                "Alpha value",
                alphaValue);
        this.isDetected = (alphaValue >= 0.07);
        return this.isDetected;
    }



    @Override
    public void run() {
        while(shouldRun) {
            this.isDetected = isObjectDetected();
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
