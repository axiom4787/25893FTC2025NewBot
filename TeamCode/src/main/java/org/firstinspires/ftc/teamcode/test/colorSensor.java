package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Detect (Purple/Green)", group = "Test")
public class colorSensor extends LinearOpMode {
    private RevColorSensorV3 sensorL;
    private RevColorSensorV3 sensorR;


    @Override
    public void runOpMode() {
        sensorL = hardwareMap.get(RevColorSensorV3.class, "sensorL"); //I2C 2
        sensorR = hardwareMap.get(RevColorSensorV3.class, "sensorR"); //I2C 3
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            int leftSensor = isColor(sensorL);
            int rightSensor = isColor(sensorR);
            int finalColor = isFinalColor(leftSensor,rightSensor);

            telemetry.addData("finalColor", finalColor);
            telemetry.update();
        }
    }
    private int isFinalColor(int ls, int rs){
        int finalColor = 0;
        if (ls == 0 && rs == 0) {
            finalColor = 0;
        } else if (ls == 2 || rs == 2) {
            finalColor = 2; // Green
        } else if (ls == 1 || rs == 1){
            finalColor = 1; // Purple
        }
        return finalColor;
    }
    private int isColor(RevColorSensorV3 colorSensor){
        float hsvValues[] = new float[3];
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
        int detectedColor;
        if (distance < 8.0) {
            if (isGreen(hue, sat, val)) {
                detectedColor = 2; // Green
            } else if (isPurple(hue, sat, val)) {
                detectedColor = 1; // Purple
            } else {
                detectedColor = 0;
            }
        } else {
            detectedColor = 0;
        }
        return detectedColor;
    }
    private boolean isGreen(float hue, float sat, float val) {
        return hue >= 80 && hue <= 190 && sat > 0.3 && val > 0.2;
    }
    private boolean isPurple(float hue, float sat, float val) {
        return (hue >= 200 && hue <= 300) && sat > 0.3 && val > 0.2;
    }
}