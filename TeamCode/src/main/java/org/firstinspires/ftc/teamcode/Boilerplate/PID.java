package org.firstinspires.ftc.teamcode.Boilerplate;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    // P, I, D coefficients
    public double kP;
    public double kI;
    public double kD;

    // PID state
    public double integral;
    public double previousError;
    public double previousTime;
        // PID output limits
    public double minOutput;
    public double maxOutput;
    private ElapsedTime time;
    public PID(double kP, double kI, double kD, double minOutput, double maxOutput) {
        time = new ElapsedTime();

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
        this.previousTime = time.milliseconds();
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }
    public double calculate(double setpoint, double actual) {
        double currentTime = time.milliseconds();
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        double error = setpoint - actual;
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        // Clamp output between minOutput and maxOutput
        if (output > maxOutput) output = maxOutput;
        if (output < minOutput) output = minOutput;
        return output;
    }
}
