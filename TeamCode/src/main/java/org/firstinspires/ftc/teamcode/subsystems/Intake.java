package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.Hardware;

public class Intake {
    public static class Power {
        public static final double OFF = 0.0;
        public static final double FORWARD = 1.0;
        public static final double REVERSE = -1.0;
    }

    private final MotorEx intakeMotor, indexerMotor;

    public Intake() {
        intakeMotor = Hardware.getIntakeMotor();
        indexerMotor = Hardware.getIndexerMotor();
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
    
    public void runIndexerIn() {
        indexerMotor.set(Power.FORWARD);
    }

    public void runIndexerOut() {
        indexerMotor.set(Power.REVERSE);
    }

    public void stopIndexer() {
        indexerMotor.set(Power.OFF);
    }
}
