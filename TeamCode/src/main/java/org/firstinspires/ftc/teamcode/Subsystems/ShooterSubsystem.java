package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class ShooterSubsystem {
    Config config = new Config();
    DcMotor shooterMotor;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        config.init(hardwareMap);

        shooterMotor = config.shooter;
    }

    public DcMotor getShooterMotor() {
        return shooterMotor;
    }
}