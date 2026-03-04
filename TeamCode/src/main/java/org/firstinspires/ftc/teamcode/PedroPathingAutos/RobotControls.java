package org.firstinspires.ftc.teamcode.PedroPathingAutos;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class RobotControls {
    Config config;
    public long shootTime = 3500;
    public long farShootTime = 6000;
    public long farRevTime = 1500; // extra time to let the shooter rev up
    public long farRevTime2 = 1000;
    public RobotControls(Config config) {
        this.config = config;
    }

    public void enableScoring() {
        config.intake.setPower(1);
        config.indexer.setPower(1);
    }

    public void enableScoringFar() {
        config.intake.setPower(0.75);
        config.indexer.setPower(0.35);
    }

    public void disableScoring() {
        config.intake.setPower(0);
        config.indexer.setPower(0);
    }

    public void enableIntake() {
        config.intake.setPower(1);
        config.indexer.setPower(-1);
    }

    public void disableIntake() {
        config.intake.setPower(0);
        config.indexer.setPower(0);
    }

    public void enableShooter() {
        config.smartShooter.setVelocity(1350);
        config.linearActuator.setPosition(0.45);
        config.turretServoLeft.setPower(0.0);
    }

    public void enableShooterFar() {
        config.smartShooter.setPower(1.0); // make it 1 so it hits all the shots
        config.linearActuator.setPosition(0.25);
        config.turretServoLeft.setPower(0.0);
    }

    public void disableShooter() {
        config.smartShooter.setPower(0);
    }
}
