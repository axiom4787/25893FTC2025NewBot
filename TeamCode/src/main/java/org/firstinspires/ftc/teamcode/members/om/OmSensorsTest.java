package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class OmSensorsTest extends OpMode {

    TouchSensor touchSensor;
    DistanceSensor distanceSensor;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

    }

    @Override
    public void loop() {
    telemetry.addData("Is Touch Sensor Pressed", touchSensor.isPressed());
    telemetry.addData("Object Distance From Sensor in cm", distanceSensor.getDistance(DistanceUnit.CM));
    telemetry.update();

    }

}
