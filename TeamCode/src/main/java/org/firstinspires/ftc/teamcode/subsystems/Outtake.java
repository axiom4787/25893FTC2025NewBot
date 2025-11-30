package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Initialize in either RPM or Power control mode
 * Change mode with public mode variable
 * You can use set function in both modes - in RPM mode set to an RPM and in power mode set to a power value (0.0-1.0)
 */

@Config
public class Outtake {

    public enum Mode {
        POWER,
        RPM
    }

    private final MotorEx shooter;
    private final RpmController controller;

    private double motorPower = 0.0;
    private double currentRPM = 0.0;
    private double targetRPM = 0.0;

    private final double turnsPerRev = 28.0;

    // Dashboard tunable gains
    public static double p = 0.0002;
    public static double s = 0.0;
    public static double v = 0.0;

    // Current mode
    public Mode mode;

    public Outtake(HardwareMap hardwareMap, Mode mode) {
        shooter = new MotorEx(hardwareMap, "outtake");
        shooter.setInverted(true);
        this.mode = mode;
        controller = new RpmController(p, s, v);
    }

    public void stop() {
        shooter.stopMotor();
        motorPower = 0.0;
        targetRPM = 0.0;
    }

    //unified control method
    public void set(double x) {
        if (mode == Mode.POWER) {
            motorPower = clamp(x, 0.0, 1.0);
        } else { // PIDF_CONTROL
            targetRPM = x;
        }
    }

    public double getRPM() {
        return currentRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getPower() {
        return motorPower;
    }

    public void periodic() {
        // Update measured RPM
        currentRPM = shooter.getVelocity() / turnsPerRev * 60.0;

        if (mode == Mode.POWER) {
            // Direct open-loop
            shooter.set(motorPower);
            return;
        }

        //control loop rpm control
        motorPower = controller.update(targetRPM, currentRPM);
        motorPower = clamp(motorPower, 0, 1.0);
        shooter.set(motorPower);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}