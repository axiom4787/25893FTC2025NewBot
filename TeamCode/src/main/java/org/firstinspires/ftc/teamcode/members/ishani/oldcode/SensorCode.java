package org.firstinspires.ftc.teamcode.members.ishani.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Ishani Sensor Code", group = "Sensor")
@Disabled
public class SensorCode extends LinearOpMode {
    TouchSensor touchSensor1;  // Touch sensor Object
    TouchSensor touchSensor2;
    @Override
    public void runOpMode() {

        // get a reference to our touchSensor object.
        touchSensor1 = hardwareMap.get(TouchSensor.class, "sensor_touch");
        touchSensor2 = hardwareMap.get(TouchSensor.class, "sensor_touch");


        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            if (touchSensor1.isPressed() || touchSensor2.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            } else {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }
            telemetry.update();
        }
    }
}