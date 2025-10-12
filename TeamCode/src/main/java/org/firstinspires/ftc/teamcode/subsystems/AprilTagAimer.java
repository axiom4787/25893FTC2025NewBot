package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AprilTagAimer {
    private final IMU imu;
    private final DcMotor leftFront, rightFront, leftBack, rightBack;
    private final double kP = 0.01; // Full PID can be done later
    private Double targetAngle = null;

    public AprilTagAimer(HardwareMap hardwareMap) {
        Movement movement = new Movement(hardwareMap);
        leftFront = movement.getLeftFront();
        rightFront = movement.getRightFront();
        leftBack = movement.getLeftBack();
        rightBack = movement.getRightBack();
        imu = movement.getImu();
    }

    private double angleWrapDegrees(double angle) {
        return (angle + 180) % 360 - 180;
    }

    public void startTurnToAprilTag(double bearing) {
        double currentYaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        targetAngle = angleWrapDegrees(currentYaw + bearing);
    }

    public double updateTurn() {
        if (targetAngle == null) return 0;

        double currentYaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        double error = angleWrapDegrees(targetAngle - currentYaw);

        double power = kP * error;
        power = Math.max(-1, Math.min(1, power));

        return power;
    }

    public boolean checkIfComplete() {
        if (targetAngle == null) return true;

        double currentYaw = imu.getRobotOrientation(
            AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES
        ).firstAngle;

        double error = angleWrapDegrees(targetAngle - currentYaw);

        // within 1 degree
        if (Math.abs(error) < 1) {
            targetAngle = null;
            return true;
        }

        return false;
    }
}
