package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class IntakeSubsystem {
    Config config = new Config();
    DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        config.init(hardwareMap);

        intakeMotor = config.intake;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public DcMotor getIntakeMotor() {
        return intakeMotor;
    }
}
