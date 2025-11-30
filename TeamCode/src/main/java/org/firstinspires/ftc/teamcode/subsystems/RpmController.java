package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class RpmController {

    // Gains
    private final double Kp;
    private final double kS; //static
    private final double kV; //velocity proportional

    public RpmController(double Kp, double kS, double kV) {
        this.Kp = Kp;
        this.kS = kS;
        this.kV = kV;
    }

    /**
     * Update FF + P controller and return motor power
     *
     * @param setpointRPM desired shooter RPM
     * @param measuredRPM current shooter RPM
     * @return motor power (-1 to 1)
     */
    public double update(double setpointRPM, double measuredRPM) {
        double error = setpointRPM - measuredRPM;

        // Feedforward
        double ff = kS * Math.signum(setpointRPM) + kV * setpointRPM;

        // P control
        double pOut = Kp * error;

        // Clamp output
        double output = clamp(ff + pOut, -1.0, 1.0);
        return output;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
