package org.firstinspires.ftc.teamcode.FRLib.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSensorW {
    private final TouchSensor touchSensor;
    
    public TouchSensorW(HardwareMap hardwareMap, String name) {
        touchSensor = hardwareMap.get(TouchSensor.class, name);
    }

    public double getValue() {
        return touchSensor.getValue();
    }
    
    public boolean isPressed() {
        return touchSensor.isPressed();
    }
}
