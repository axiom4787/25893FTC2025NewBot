package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.util.Context;

public class Lights {
    private final RevBlinkinLedDriver lights;

    public Lights() {
        lights = Context.getLights();
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        lights.setPattern(pattern);
    }
}
