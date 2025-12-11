package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class ledstuff {
    private LED redLed;
    private LED greenLed;

    public void init(HardwareMap hwMap){
        redLed = hwMap.get(LED.class,"rLed");
        greenLed = hwMap.get(LED.class,"gLed");

    }
    public void setRedLed(boolean isOn){
        if(isOn) {
            redLed.on();
        }
        else{
            redLed.off();
        }
    }
    public void setGreenLed(boolean isOn){
        if(isOn) {
            greenLed.on();
        }
        else{
            greenLed.off();
        }
    }
}
