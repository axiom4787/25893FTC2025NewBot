package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Globals;


public class Hood {
    private static class PhysicalPosition {
        public static final double DOWN = 0.75;
        public static final double UP = 0.15;
    }

    public static class Position {
        public static final double DOWN = 0.0;
        public static final double MIDDLE = 0.5;
        public static final double UP = 1.0;
    }

    private final ServoEx hoodActuator;

    public Hood() {
        hoodActuator = Context.getHoodActuator();
    }

    public void update() {
        if (!Globals.isNearLaunchZone()) return;
        // Don't do anything if not close to a launch zone.

        double distFromGoal = Globals.distToGoal();

        // 20in  - up against goal
        // 60in  - mid close launch zone, max hood farther than here
        // 90in  - tip of close launch zone triangle
        // 125in - front of far launch zone
        // 140in - back of far launch zone

        if (distFromGoal > 100) {
            // Far launch zone
            setHoodAngle(Position.UP);
        } else if (distFromGoal > 80) {
            // Edge of close launch zone
            setHoodAngle(0.75);
        } else if (distFromGoal < 25) {
            // Super close to goal
            setHoodAngle(0.25);
        } else {
            double pct = Range.scale(distFromGoal, 25, 80, 0.25, 0.75);
            // Linear interpolation from distance to hood angle.
            // 25 -> 0.25, 80 -> 0.75, etc
            setHoodAngle(pct);
        }
    }

    /**
     * @param angle 0: Down, 1: Up
     */
    private void setHoodAngle(double angle) {
        double clippedAngle = Range.clip(angle, 0, 1);

        // turn the [0, 1] range of inputs into [0.75, 0.25] of the hood
        double physicalPosition = Range.scale(clippedAngle, Position.DOWN, Position.UP, PhysicalPosition.DOWN, PhysicalPosition.UP);

        hoodActuator.set(physicalPosition);
    }

    public double getAngle() {
        double pos = hoodActuator.get();

        return Range.scale(pos, PhysicalPosition.DOWN, PhysicalPosition.UP, Position.DOWN, Position.UP);
    }
}
