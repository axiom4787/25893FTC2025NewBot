package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "IS - Distance Sensor Beginner", group = "ISSensor")
public class DistanceSensorBeginner extends LinearOpMode {

    // ←←← Name this EXACTLY what you named it in the phone config!
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        // ———————— 1. CONNECT THE SENSOR ————————
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");  // change "distance" if yours is named differently

        telemetry.addData("DISTANCE SENSOR READY", "Point me at something!");
        telemetry.addData("I will show distance in INCHES and CM", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ———————— 2. READ THE DISTANCE ————————
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            double distanceINCH = distanceSensor.getDistance(DistanceUnit.INCH);

            // ———————— 3. SHOW IT ON THE PHONE ————————
            telemetry.addData("Distance", "%.1f cm  |  %.1f inches", distanceCM, distanceINCH);

            // ———————— 4. FUN EXAMPLES ————————
            if (distanceINCH < 6) {
                telemetry.addData("TOO CLOSE!", "Back up!");
            } else if (distanceINCH < 12) {
                telemetry.addData("PERFECT", "You are 6–12 inches away!");
            } else {
                telemetry.addData("Far away", "Come closer :)");
            }

            telemetry.update();
        }
    }
}