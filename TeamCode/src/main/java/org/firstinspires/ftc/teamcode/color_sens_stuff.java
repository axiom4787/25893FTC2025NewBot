package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class color_sens_stuff {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        PURP,
        GREEN,
        BLUE,
        VOID;
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sens1");
        colorSensor.setGain(15);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;
        float totalBrightness = normRed + normGreen + normBlue;

        telemetry.addData("R", "%.3f", normRed);
        telemetry.addData("G", "%.3f", normGreen);
        telemetry.addData("B", "%.3f", normBlue);
        telemetry.addData("Total Brightness", "%.3f", totalBrightness);

        // Simple brightness-based approach:
        if (totalBrightness > 1.2f) {        // Very bright = NOTHING
            return DetectedColor.VOID;
        } else if (totalBrightness > 0.6f) { // Medium bright = GREEN
            return DetectedColor.GREEN;
        } else {                             // Dark = PURPLE
            return DetectedColor.PURP;
        }
    }
}