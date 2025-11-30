package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Outtake {
    private final MotorEx shooter;

    private double motorPower = 0.0;
    public static int targetRPM = 0;
    private double currentRPM = 0.0;
    private final RpmPIDF rpmPIDF;

    // Shooter state
    public boolean shooterEnabled = false;

    // Encoder ticks per rotation
    private final double TPR = 28.0;

    public static double p = 0.0002;
    public static double s = 0.0;
    public static double v = 0.0;

    public Outtake(HardwareMap hardwareMap) {
        shooter = new MotorEx(hardwareMap, "outtake");
        shooter.setInverted(true);

        // Initialize PIDF controller
        rpmPIDF = new RpmPIDF(p,s,v);
    }

    public void stop() {
        shooter.stopMotor();
        motorPower = 0.0;
    }

    public void setPower(double power) {
        shooter.set(motorPower);
    }

    public void setTargetRPM(int rpm) {
        targetRPM = rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return currentRPM;
    }

    public double getPower() {
        return motorPower;
    }

    //call periodically to keep the pidf goin
    public void periodic() {
        if (!shooterEnabled) return;

        // Read current motor velocity in ticks/sec, convert to RPM
        currentRPM = shooter.getVelocity() / TPR * 60.0;

        // Update PIDF controller
        motorPower = rpmPIDF.update(targetRPM, currentRPM);

        // Clamp and apply power
        motorPower = clamp(motorPower, 0, 1.0);
        shooter.set(motorPower);
    }

    //utility for clamping
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
