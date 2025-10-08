package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "HSV Color Detect (Purple/Green)", group = "Test")
public class colorSensor extends LinearOpMode {
    private RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor"); // I2C Bus 1
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        float hsvValues[] = new float[3];
        while (opModeIsActive()) {
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            // Scale RGB to 0–255 and convert to HSV
            Color.RGBToHSV(r * 255 / 800, g * 255 / 800, b * 255 / 800, hsvValues);
            float hue = hsvValues[0];  // Hue (0 to 360)
            float sat = hsvValues[1];  // Saturation (0 to 1)
            float val = hsvValues[2];  // Value (0 to 1)
            // Get distance
            double distance = colorSensor.getDistance(DistanceUnit.MM);
            // Color classification
            String detectedColor = "Unknown";
            if (distance < 25.0) {
                if (isGreen(hue, sat, val)) {
                    detectedColor = "Green";
                } else if (isPurple(hue, sat, val)) {
                    detectedColor = "Purple";
                } else {
                    detectedColor = "Other Color";
                }
            } else {
                detectedColor = "Nothing Nearby";
            }
            // Telemetry
            telemetry.addData("Distance (mm)", "%.2f", distance);
            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("Sat", "%.2f", sat);
            telemetry.addData("Val", "%.2f", val);
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
        }
    }
    private boolean isGreen(float hue, float sat, float val) {
        return hue >= 80 && hue <= 190 && sat > 0.3 && val > 0.2;
    }
    private boolean isPurple(float hue, float sat, float val) {
        return (hue >= 200 && hue <= 300) && sat > 0.3 && val > 0.2;
    }
}