package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class IntakeSubsystem {
    public enum State { OFF, FORWARD, REVERSE }
    public static class Power {
        public static final double OFF = 0.0;
        public static final double FORWARD = 1.0;
        public static final double REVERSE = -1.0;
    }
    State state = State.OFF;

    DcMotor intakeMotor;

    public IntakeSubsystem(Config config) {
        intakeMotor = config.intake;
    }

    public void update() {
        double power;

        switch (state) {
            case FORWARD:
                power = Power.FORWARD;
                break;
            case REVERSE:
                power = Power.REVERSE;
                break;
            case OFF:
            default:
                power = Power.OFF;
        }

        setPower(power);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public DcMotor getIntakeMotor() {
        return intakeMotor;
    }
}
