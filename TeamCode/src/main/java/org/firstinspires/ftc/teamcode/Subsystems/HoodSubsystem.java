package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class HoodSubsystem {
    Config config = new Config();
    Servo hoodActuator;

    public HoodSubsystem(HardwareMap hardwareMap) {
        config.init(hardwareMap);

        hoodActuator = config.linearActuator;
    }

    public void setActuatorPosition(double position) {
        // TODO: Do math to turn the [0, 1] / [fully retracted, fully extended] range
        //  of the actuator into a [0, 1] / [hood down, hood up] range because that kinda
        //  makes more sense for what we're doing

        hoodActuator.setPosition(position);
    }

    public Servo getHoodActuator() {
        return hoodActuator;
    }
}
