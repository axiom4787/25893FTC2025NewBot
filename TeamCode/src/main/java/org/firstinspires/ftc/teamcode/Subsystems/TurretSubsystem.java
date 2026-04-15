package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.CachedRTPAxon;
import org.firstinspires.ftc.teamcode.Boilerplate.Shared;
import org.firstinspires.ftc.teamcode.Hardware.CachingHardware;

public class TurretSubsystem {
    public final CachedRTPAxon turretController;
    public static final double GEAR_RATIO = 98 / 35f;
    public static final double MAX_SERVO_ANGLE_RIGHT = GEAR_RATIO * 65;
    public static final double MAX_SERVO_ANGLE_LEFT  = GEAR_RATIO * 120;

    public TurretSubsystem() {
        turretController = CachingHardware.getTurretServo();
    }

    // TODO: Custom PID accounting for wraparound, like RTPAxon for CRServoEx
    //  because CRServoEx doesn't seem to be able to handle unnormalized angles
    public void update(Follower follower) {
        double robotHeading = follower.getHeading();

        double diffX = Shared.Misc.GOAL_X - follower.getPose().getX();
        double diffY = Shared.Misc.GOAL_Y - follower.getPose().getY();
        double headingTowardsGoal = Math.atan2(diffY, diffX);

        double targetTurretAngle = MathFunctions.normalizeAngle(headingTowardsGoal - robotHeading);
        if (targetTurretAngle > Math.PI) targetTurretAngle -= 2 * Math.PI; // get angle in the range of [-pi, pi]

        targetTurretAngle = Math.toDegrees(targetTurretAngle) * GEAR_RATIO; // convert to degrees and account for gear ratio
        targetTurretAngle = Range.clip(targetTurretAngle, -MAX_SERVO_ANGLE_RIGHT, MAX_SERVO_ANGLE_LEFT); // limit how far the turret can go

        turretController.setTargetRotation(targetTurretAngle);
        turretController.update();
    }

    public double getCurrentAngle() {
        return turretController.getTotalRotation();
    }

    public double getTargetAngle() {
        return turretController.getTargetRotation();
    }
}
