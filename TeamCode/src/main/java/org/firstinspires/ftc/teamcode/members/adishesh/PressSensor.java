package org.firstinspires.ftc.teamcode.members.adishesh;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Adi's touch sensor")
@Disabled
public class PressSensor extends LinearOpMode {

    TouchSensor touchSensor;  // Touch sensor Object

    @Override
    public void runOpMode() {

touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");


        waitForStart();

        while (opModeIsActive()) {
            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is pressed");
            } else {
                telemetry.addData("Touch sensor", "is not pressed");
            }

        }
    }
}
