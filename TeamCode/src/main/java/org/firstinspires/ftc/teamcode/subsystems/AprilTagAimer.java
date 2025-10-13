package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AprilTagAimer {
    private final double kP = 0.01;
    private final double kI = 0.001;
    private final double kD = 0.005;
    private final double kF = 0.05;
    private final double maxIntegral = 1.0;

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
    }

    private double angleWrapDegrees(double angle) {
        return (angle + 180) % 360 - 180;
    }

    public double calculateTurnPowerToBearing(double bearing) {
        // If apriltag lost - reset PID state and return no correction
        if (Double.isNaN(bearing)) {
            integral = 0;
            lastError = 0;
            lastTimestamp = 0;
            return 0.0;
        }

        double error = -angleWrapDegrees(bearing);

        // Time diff in seconds
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTimestamp) / 1000.0;

        if (lastTimestamp == 0) {
            deltaTime = 0.01;  // assume 10ms for first call
        } else {
            deltaTime = (currentTime - lastTimestamp) / 1000.0;
        }

        lastTimestamp = currentTime;


        // Integral accumulation
        integral += error * deltaTime;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        // Gets proper direction
        double feedforward = Math.signum(error) * kF;

        // PIDF output
        double power = kP * error + kI * integral + kD * derivative + feedforward;

        return Math.max(-1, Math.min(1, power));
    }
}
