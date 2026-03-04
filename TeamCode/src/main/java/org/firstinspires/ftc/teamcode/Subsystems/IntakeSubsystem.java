package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class IntakeSubsystem {
    public static class Power {
        public static final double OFF = 0.0;
        public static final double FORWARD = 1.0;
        public static final double REVERSE = -1.0;
    }

    private final DcMotor intakeMotor;

    public IntakeSubsystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}
