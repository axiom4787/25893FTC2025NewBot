package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private final MotorEx motor;
    private double rpm;
    double lastTime;

    public static class Params
    {
        public static final double DISTANCE_TO_RPM_COEFF = 0.8;
    }

    public Outtake(HardwareMap hardwareMap)
    {
        motor = new MotorEx(hardwareMap, "outtake");
        // set the run mode
        motor.setRunMode(Motor.RunMode.VelocityControl);

        // set and get the coefficients
        motor.setVeloCoefficients(0.05, 0.01, 0.31);
        motor.setFeedforwardCoefficients(0.92, 0.47);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        lastTime = System.nanoTime();
    }

    public void stop()
    {
        motor.stopMotor();
    }

    public void run()
    {
        motor.setVelocity(rpmToTicksPerSecond(rpm));
    }

    public void setRPM(double newPower)
    {
        rpm = newPower;
    }

    public double getRPM()
    {
        return rpm;
    }

    public double getMeasuredRPM()
    {
        double ticksPerSecond = motor.get();
        double ticksPerRev = motor.getCPR();
        double revsPerSecond = ticksPerSecond / ticksPerRev;
        return revsPerSecond * 60.0;
    }

    public double rpmToTicksPerSecond(double rpm) {
        double ticksPerRev = motor.getCPR(); // counts per revolution
        return (rpm / 60.0) * ticksPerRev;
    }

    public void setOuttakeVelocityFromDistance(double distanceInInches) {
        // Simple model: farther target = higher velocity
        // You’ll replace this with a better fit (quadratic or regression later)
        double targetRPM = Params.DISTANCE_TO_RPM_COEFF * Math.sqrt(distanceInInches);

        // Convert to ticks/sec
        double targetVelocityTicks = rpmToTicksPerSecond(targetRPM);

        // Set the velocity target
        motor.setVelocity(targetVelocityTicks);

        // if we wanna debug later
        //telemetry.addData("Target Distance (in)", distanceInInches);
        //telemetry.addData("Target RPM", targetRPM);
        //telemetry.addData("Target Velocity (tps)", targetVelocityTicks);
    }
    //TODO: Make algorithm to determine power needed based on distance
}
