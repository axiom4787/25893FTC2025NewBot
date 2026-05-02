package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.Context;

public class Intake {
    public static class Power {
        public static final double OFF = 0.0;
        public static final double FORWARD = 1.0;
        public static final double REVERSE = -1.0;
    }

    private enum State { OFF, IN, OUT, INDEX }

    private State state = State.OFF;

    private final MotorEx intakeMotor, indexerMotor;

    public Intake() {
        intakeMotor = Context.getIntakeMotor();
        indexerMotor = Context.getIndexerMotor();
    }

    public void intake() {
        state = State.IN;
    }

    public void index() {
        state = State.INDEX;
    }

    public void reverse() {
        state = State.OUT;
    }

    public void off() {
        state = State.OFF;
    }

    private void update() {
        switch (state) {
            case IN:
                intakeMotor.set(Power.FORWARD);
                indexerMotor.set(Power.REVERSE);
                break;
            case OUT:
                intakeMotor.set(Power.REVERSE);
                indexerMotor.set(Power.REVERSE);
                break;
            case INDEX:
                intakeMotor.set(Power.FORWARD);
                indexerMotor.set(Power.FORWARD);
                break;
            case OFF:
                intakeMotor.set(Power.OFF);
                indexerMotor.set(Power.OFF);
                break;
        }
    }
}
