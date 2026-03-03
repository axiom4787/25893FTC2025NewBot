package org.firstinspires.ftc.teamcode.PedroPathingAutos;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class RobotControls {
    Config config;
    public double shootTime = 3.5;
    public double farShootTime = 6.0;
    public RobotControls(Config config) {
        this.config = config;
    }

    public void enableScoring() {
        config.intake.setPower(1);
        config.indexer.setPower(1);
    }

    public void enableScoring(double intake, double indexer) {
        config.intake.setPower(intake);
        config.indexer.setPower(indexer);
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
        enableShooter(1350, 0.45);
    }

    public void enableShooter(double vel, double pos) {
        config.smartShooter.setVelocity(vel);
        config.linearActuator.setPosition(pos);
        config.turretServoLeft.setPower(0.0);
    }

    public void disableShooter() {
        config.smartShooter.setPower(0);
    }
}
