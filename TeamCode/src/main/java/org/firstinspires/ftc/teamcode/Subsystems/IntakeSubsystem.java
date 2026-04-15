package org.firstinspires.ftc.teamcode.Subsystems;

import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Hardware.CachingHardware;

public class IntakeSubsystem {
    public static class Power {
        public static final double OFF = 0.0;
        public static final double FORWARD = 1.0;
        public static final double REVERSE = -1.0;
    }

    private final MotorEx intakeMotor, indexerMotor;

    public IntakeSubsystem() {
        intakeMotor = CachingHardware.getIntakeMotor();
        indexerMotor = CachingHardware.getIndexerMotor();
    }

    public void intake() {
        runIntakeIn();
        runIndexerOut();
    }

    public void index() {
        runIntakeIn();
        runIndexerIn();
    }

    public void reverse() {
        runIntakeOut();
        runIndexerOut();
    }

    public void off() {
        stopIntake();
        stopIndexer();
    }

    public void runIntakeIn() {
        intakeMotor.set(Power.FORWARD);
    }

    public void runIntakeOut() {
        intakeMotor.set(Power.REVERSE);
    }

    public void stopIntake() {
        intakeMotor.set(Power.OFF);
    }

    public void runIntake(double power) {
        intakeMotor.set(power);
    }

    public void runIndexerIn() {
        indexerMotor.set(Power.FORWARD);
    }

    public void runIndexerOut() {
        indexerMotor.set(Power.REVERSE);
    }

    public void stopIndexer() {
        indexerMotor.set(Power.OFF);
    }

    public void runIndexer(double power) {
        indexerMotor.set(power);
    }
}
