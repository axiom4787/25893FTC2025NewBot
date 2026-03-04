package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class HoodSubsystem {
    public static class Position {
        public static final double DOWN = 0.75;
        public static final double MIDDLE = 0.5;
        public static final double UP = 0.25;
    }

    private final Servo hoodActuator;

    public HoodSubsystem(Servo hoodActuator) {
        this.hoodActuator = hoodActuator;
    }

    public void setActuatorPositionConstrained(double position) {
//         TODO: Do math to turn the [0, 1] / [fully retracted, fully extended] range
//          of the actuator into a [0, 1] / [hood down, hood up] range because that kinda
//          makes more sense for what we're doing maybe?

        if (position > Position.DOWN)   position = Position.DOWN;
        if (position < Position.UP)     position = Position.UP;

        hoodActuator.setPosition(position);
    }
}
