package org.firstinspires.ftc.teamcode.mechanism;

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
