package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.Boilerplate.Shared;
import org.firstinspires.ftc.teamcode.Hardware.CachingHardware;

public class HoodSubsystem {
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

    public HoodSubsystem() {
        hoodActuator = CachingHardware.getHoodActuator();
    }

    public void update(Follower follower) {
        double distX = Shared.Misc.GOAL_X - follower.getPose().getX();
        double distY = Shared.Misc.GOAL_Y - follower.getPose().getY();
        double distFromGoal = Math.hypot(distX, distY);

        // 20in  - up against goal
        // 60in  - mid close launch zone, max hood farther than here
        // 90in  - tip of close launch zone triangle
        // 125in - front of far launch zone
        // 140in - back of far launch zone

        if (distFromGoal > 100) {
            setHoodAngle(Position.UP);
        } else if (distFromGoal < 25) {
            setHoodAngle(0.25);
        } else {
            double pct = Range.scale(distFromGoal, 25, 100, 0.25, 1);
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
}
