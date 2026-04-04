package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterSubsystem {
    private final DcMotorEx shooter;

    public ShooterSubsystem() {
        shooter = Hardware.getShooterMotor();
    }

    public void setShooterVelocity(double velocity) {
        shooter.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        return shooter.getVelocity();
    }

    public void stopShooter() {
        shooter.setPower(0.0);
    }
}