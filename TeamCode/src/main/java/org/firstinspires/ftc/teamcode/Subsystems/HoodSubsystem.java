package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class HoodSubsystem {
    public enum State { OFF, UP, MIDDLE, DOWN, AUTO }
    public State state = State.OFF;
    public static class Position {
        public static final double DOWN = 0.75;
        public static final double MIDDLE = 0.5;
        public static final double UP = 0.25;
    }

    Servo hoodActuator;
    LimeLightSubsystem limeLightSubsystem;

    public HoodSubsystem(Config config, LimeLightSubsystem limeLightSubsystem) {
        this.limeLightSubsystem = limeLightSubsystem;
        hoodActuator = config.linearActuator;
    }

    double autoLinearActuatorValue = 0.0;
    public void update() {
        double actuatorPosition = hoodActuator.getPosition();
        switch (state) {
            case AUTO:
                LLResult target = limeLightSubsystem.getTarget();
                if (target == null) break;

                autoLinearActuatorValue = limeLightSubsystem.calculateHood(target);
                actuatorPosition -= autoLinearActuatorValue;
                hoodActuator.setPosition(actuatorPosition);
                break;

            case UP:
                setActuatorPosition(Position.UP);
                break;

            case MIDDLE:
                setActuatorPosition(Position.MIDDLE);
                break;

            case DOWN:
                setActuatorPosition(Position.DOWN);
                break;
        }
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
