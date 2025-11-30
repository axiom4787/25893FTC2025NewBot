package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class RpmController {

    // Gains
    private final double Kp;
    private final double kS; //static
    private final double kV; //velocity proportional

    // State
    private double lastMeasurement = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    public RpmController(double Kp, double kS, double kV) {
        this.Kp = Kp;
        this.kS = kS;
        this.kV = kV;
        timer.reset();
    }

    /**
     * Update FF + P controller and return motor power
     *
     * @param setpointRPM desired shooter RPM
     * @param measuredRPM current shooter RPM
     * @return motor power (-1 to 1)
     */
    public double update(double setpointRPM, double measuredRPM) {
        double dt = Math.max(timer.seconds(), 1e-6);
        timer.reset();

        double error = setpointRPM - measuredRPM;

        // Feedforward
        double ff = kS * Math.signum(setpointRPM) + kV * setpointRPM;

        // P control
        double pOut = Kp * error;

        // Clamp output
        double output = clamp(ff + pOut, -1.0, 1.0);

        lastMeasurement = measuredRPM;
        return output;
    }

    public void reset() {
        lastMeasurement = 0.0;
        timer.reset();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
