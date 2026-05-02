package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.Globals.Poses.*;
import static org.firstinspires.ftc.teamcode.util.Globals.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.CachedRTPAxon;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Globals;

public class Turret {
    public final CachedRTPAxon turretController;
    public static final double GEAR_RATIO = 98 / 35f;
    public static final double MAX_SERVO_ANGLE_RIGHT = GEAR_RATIO * 65;
    public static final double MAX_SERVO_ANGLE_LEFT  = GEAR_RATIO * 120;

    public Turret() {
        turretController = Context.getTurretServo();
    }

    public void update() {
        if (!Globals.isNearLaunchZone()) {
            turretController.setTargetRotation(0);
            turretController.update();
            return;
        } // Center turret if not close to a launch zone.

        Pose robotPose = Context.follower.getPose();

        double robotHeading = robotPose.getHeading();

        double turretOffset = 3.0; // 3 inches behind center of robot
        double turretX = robotPose.getX() - turretOffset * Math.cos(robotPose.getHeading());
        double turretY = robotPose.getY() - turretOffset * Math.sin(robotPose.getHeading());

        double diffX = pose(GOAL).getX() - turretX;
        double diffY = pose(GOAL).getY() - turretY;
        double headingTowardsGoal = Math.atan2(diffY, diffX);
        // Get angle from center of turret to goal

        double targetTurretAngle = MathFunctions.normalizeAngleSigned(headingTowardsGoal - robotHeading);
        // Get angle in the range of [-pi, pi]

        targetTurretAngle = Math.toDegrees(targetTurretAngle) * GEAR_RATIO; // convert to degrees and account for gear ratio
        targetTurretAngle = Range.clip(targetTurretAngle, -MAX_SERVO_ANGLE_RIGHT, MAX_SERVO_ANGLE_LEFT); // limit how far the turret can go

        turretController.setTargetRotation(targetTurretAngle);
        turretController.update();
    }

    public double getCurrentServoAngle() {
        return turretController.getTotalRotation();
    }

    public double getTargetServoAngle() {
        return turretController.getTargetRotation();
    }

    public double getCurrentAngle() {
        return getCurrentServoAngle() * GEAR_RATIO;
    }

    public double getTargetAngle() {
        return getTargetServoAngle() * GEAR_RATIO;
    }
}
