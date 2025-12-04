package org.firstinspires.ftc.teamcode.FRLib.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance2mW {
    DistanceSensor distanceSensor;

    public Distance2mW(HardwareMap hardwareMap, String name) {
        distanceSensor = hardwareMap.get(DistanceSensor.class, name);
    }

    public double getDistance(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }

    public double getDistanceInches() {
        return getDistance(DistanceUnit.INCH);
    }

    public double getDistanceMeters() {
        return getDistance(DistanceUnit.METER);
    }

    public double getDistanceCentimeters() {
        return getDistance(DistanceUnit.CM);
    }

    public double getDistanceMillimeters() {
        return getDistance(DistanceUnit.MM);
    }
}
