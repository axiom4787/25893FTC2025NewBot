package org.firstinspires.ftc.teamcode.members.bala;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Bala - Sensor")

public class  BalaSensor  extends OpMode {

    TouchSensor touchSensor;
    DistanceSensor distanceSensor;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

        @Override
        public void loop() {
        telemetry.addData("Is Touch Sensor Pressed", touchSensor.isPressed() );
        telemetry.addData("Is Distance Sensor Pressed" , distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        }


    }




