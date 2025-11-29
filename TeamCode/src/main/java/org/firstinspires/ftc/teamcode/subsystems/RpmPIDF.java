package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class RpmPIDF {
    // gains like the gym type shift
    public double Kp = 0.001;
    public double Ki = 0.0001;
    public double Kd = 0.0001;

    // ff gains
    public double kS = 0.0;   // static friction
    public double kV = 0.001; // proportional to RPM

    // state
    private double integral = 0.0;
    private double lastMeasurement = 0.0;

    private final double integralLimit = 0.5; // clamp integral

    // smoting
    private final double derivAlpha = 0.6;
    private double lastDerivativeFiltered = 0.0;

    // Timer for delta-time calculation
    private final ElapsedTime timer = new ElapsedTime();

    public RpmPIDF() {
        timer.reset();
    }

    /**
     * guys is this tuff params
      func Updates the PIDF controller and returns the motor power output.
      @param setpointRPM Desired RPM
      @param measuredRPM Current RPM
      @return Motor power command (-1.0 to 1.0)
     */
    public double update(double setpointRPM, double measuredRPM) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6; // avoid divide by zero (guys im smart right?)

        double error = setpointRPM - measuredRPM;

        // Integral clamped
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // Derivative with kickredcution
        double rawDerivative = -(measuredRPM - lastMeasurement) / dt;
        double derivativeFiltered = derivAlpha * lastDerivativeFiltered + (1 - derivAlpha) * rawDerivative;
        lastDerivativeFiltered = derivativeFiltered;

        double pidOut = Kp * error + Ki * integral + Kd * derivativeFiltered;

        // ff type shift
        double ff = kS * Math.signum(setpointRPM) + kV * setpointRPM;

        // clamped final output
        double output = clamp(ff + pidOut, -1.0, 1.0);

        lastMeasurement = measuredRPM;

        return output;
    }

    //reset state
    public void reset() {
        integral = 0.0;
        lastDerivativeFiltered = 0.0;
        lastMeasurement = 0.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
