package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.CachedRTPAxon;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Hardware;

public class Turret {
    public final CachedRTPAxon turretController;
    public static final double GEAR_RATIO = 98 / 35f;
    public static final double MAX_SERVO_ANGLE_RIGHT = GEAR_RATIO * 65;
    public static final double MAX_SERVO_ANGLE_LEFT  = GEAR_RATIO * 120;

    public Turret() {
        turretController = Hardware.getTurretServo();
    }

    public void update(Follower follower) {
        if (!Globals.Zones.isNearLaunchZone()) {
            turretController.setPower(0);
            return;
        } // Center turret if not close to a launch zone.

        double robotHeading = follower.getHeading();

        double turretOffset = 3.0; // 3 inches behind center of robot
        double turretX = follower.getPose().getX() - turretOffset * Math.cos(follower.getHeading());
        double turretY = follower.getPose().getY() - turretOffset * Math.sin(follower.getHeading());

        double diffX = Globals.Misc.GOAL_X - turretX;
        double diffY = Globals.Misc.GOAL_Y - turretY;
        double headingTowardsGoal = Math.atan2(diffY, diffX);

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
