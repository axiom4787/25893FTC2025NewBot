package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class HoodSubsystem {
    private static class PhysicalPosition {
        public static final double DOWN = 0.75;
        public static final double UP = 0.25;
    }

    public static class Position {
        public static final double DOWN = 0.0;
        public static final double MIDDLE = 0.5;
        public static final double UP = 1.0;
    }

    private final Servo hoodActuator;

    public HoodSubsystem() {
        hoodActuator = Hardware.getHoodActuator();
    }

    /**
     * @param angle 0: Down, 1: Up
     */
    public void setHoodAngle(double angle) {
        double clippedAngle = Range.clip(angle, 0, 1);

        // turn the [0, 1] range of inputs into [0.75, 0.25] of the hood
        double physicalPosition = Range.scale(angle, Position.DOWN, Position.UP, PhysicalPosition.DOWN, PhysicalPosition.UP);

        hoodActuator.setPosition(physicalPosition);
    }
}
