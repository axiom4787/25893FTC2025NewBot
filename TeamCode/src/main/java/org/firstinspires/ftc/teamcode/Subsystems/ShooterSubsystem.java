package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class ShooterSubsystem {
    DcMotor shooterMotor;

    public ShooterSubsystem(Config config) {
        shooterMotor = config.shooter;
    }

    public DcMotor getShooterMotor() {
        return shooterMotor;
    }
}