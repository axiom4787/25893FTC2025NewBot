package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AprilTagAimer {
    private final IMU imu;

    private final double kP = 0.01;
    private final double kI = 0.001;
    private final double kD = 0.005;
    private final double kF = 0.05;

    private double integral = 0;
    private double lastError = 0;
    private long lastTimestamp = 0;

    /* When and why to tune these
    P (Proportional) Changes core power of turns, its proportional
    I (Integral) Maybe rarely if robot consistently falls short of the target (steady-state error).
    D (Derivative) Increase to dampen motion and reduce overshoot. Good for smoothing quick heading corrections.
    F (Feedforward)	Maybe, its a constant, increase to help overcome drivetrain static friction and give better response when error is small.
    */
    public AprilTagAimer(HardwareMap hardwareMap) {
        Movement movement = new Movement(hardwareMap);
        imu = movement.getImu();
    }

    private double angleWrapDegrees(double angle) {
        return (angle + 180) % 360 - 180;
    }

    public double calculateTurnPowerToBearing(double bearing) {
        double currentYaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        double targetAngle = angleWrapDegrees(currentYaw + bearing);
        double error = angleWrapDegrees(targetAngle - currentYaw);

        // Time diff in seconds
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTimestamp) / 1000.0;
        lastTimestamp = currentTime;

        // Integral accumulation
        integral += error * deltaTime;

        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        // Gets proper direction
        double feedforward = Math.signum(error) * kF;

        // PID output
        double power = kP * error + kI * integral + kD * derivative + feedforward;

        return Math.max(-1, Math.min(1, power));
    }
}
