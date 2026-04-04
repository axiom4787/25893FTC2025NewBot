package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {
    public static class Power {
        public static final double OFF = 0.0;
        public static final double FORWARD = 1.0;
        public static final double REVERSE = -1.0;
    }

    private final DcMotor intakeMotor, indexerMotor;

    public IntakeSubsystem() {
        intakeMotor = Hardware.getIntakeMotor();
        indexerMotor = Hardware.getIndexerMotor();
    }

    public void intake() {
        runIndexerIn();
        runIndexerOut();
    }

    public void index() {
        runIntakeIn();
        runIndexerOut();
    }

    public void off() {
        stopIntake();
        stopIndexer();
    }

    public void runIntakeIn() {
        intakeMotor.setPower(Power.FORWARD);
    }

    public void runIntakeOut() {
        intakeMotor.setPower(Power.REVERSE);
    }

    public void stopIntake() {
        intakeMotor.setPower(Power.OFF);
    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void runIndexerIn() {
        indexerMotor.setPower(Power.FORWARD);
    }

    public void runIndexerOut() {
        indexerMotor.setPower(Power.REVERSE);
    }

    public void stopIndexer() {
        indexerMotor.setPower(Power.OFF);
    }

    public void runIndexer(double power) {
        indexerMotor.setPower(power);
    }
}
