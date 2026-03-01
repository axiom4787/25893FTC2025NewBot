package org.firstinspires.ftc.teamcode.PedroPathingAutos;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class RobotControls {
    Config config;
    public RobotControls(Config config) {
        this.config = config;
    }

    public void enableScoring() {
        config.intake.setPower(1);
        config.indexer.setPower(1);
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
        config.smartShooter.setVelocity(1500);
    }

    public void disableShooter() {
        config.smartShooter.setPower(0);
    }
}
