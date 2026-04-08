package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Boilerplate.RTPAxon;
import org.firstinspires.ftc.teamcode.NewAutos.Shared2;

public class TurretSubsystem {
    private final CRServo leftTurretServo, rightTurretServo;
    private final AnalogInput servoEncoder;
    public final RTPAxon smartServoController;
    public static final double GEAR_RATIO = 98 / 35f;
    public static final double MAX_SERVO_ANGLE_RIGHT = GEAR_RATIO * 70;
    public static final double MAX_SERVO_ANGLE_LEFT  = GEAR_RATIO * 120;

    public TurretSubsystem() {
        Hardware.TurretServos turretServos = Hardware.getTurretServos();

        leftTurretServo = turretServos.left;
        rightTurretServo = turretServos.right;
        servoEncoder = Hardware.getAxonServoEncoder();

        smartServoController = new RTPAxon(leftTurretServo, servoEncoder, RTPAxon.Direction.FORWARD);
        smartServoController.setPidCoeffs(0.013, 0.0, 0.0);
        smartServoController.setRtp(true);
    }

    public void update(Follower follower) {
        double robotHeading = follower.getHeading();

        double diffX = Shared2.Misc.GOAL_X - follower.getPose().getX();
        double diffY = Shared2.Misc.GOAL_Y - follower.getPose().getY();
        double headingTowardsGoal = Math.atan2(diffY, diffX);

        double targetTurretAngle = MathFunctions.normalizeAngle(headingTowardsGoal - robotHeading);
        if (targetTurretAngle > Math.PI) targetTurretAngle -= 2 * Math.PI; // get angle in the range of [-pi, pi]

        targetTurretAngle = Math.toDegrees(targetTurretAngle) * GEAR_RATIO; // convert to degrees and account for gear ratio
        targetTurretAngle = Range.clip(targetTurretAngle, -MAX_SERVO_ANGLE_RIGHT, MAX_SERVO_ANGLE_LEFT); // limit how far the turret can go

        smartServoController.setTargetRotation(targetTurretAngle);
        smartServoController.update();
    }

    public double getTurretAngle() {
        return smartServoController.getTotalRotation();
    }

    public double getTurretTargetAngle() {
        return smartServoController.getTargetRotation();
    }
}
