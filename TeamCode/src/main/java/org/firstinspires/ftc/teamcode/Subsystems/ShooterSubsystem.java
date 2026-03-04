package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterSubsystem {
    private final DcMotorEx smartShooter;

    public ShooterSubsystem(DcMotorEx smartShooter) {
        this.smartShooter = smartShooter;
    }

    public void setShooterVelocity(double velocity) {
        smartShooter.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        return smartShooter.getVelocity();
    }

    public void setShooterPower(double power) {
        smartShooter.setPower(power);
    }
}