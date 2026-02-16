package org.firstinspires.ftc.teamcode.Boilerplate;

public class PID {
    double kP, kI, kD;

    double P, I, D;

    double error;
    double integral = 0;
    double derivative = 0;
    double prevError = 0;
    public PID(double p, double i, double d){
        kP = p;
        kI = i;
        kD = d;
    }
    public double calculate(double setpoint, double current) {
        error = setpoint - current;
        integral += error;
        derivative = prevError - error;
        P = error * kP;
        I = integral * kI;
        D = derivative * kD;

        prevError = error;
        return P + I + D;
    }
}
