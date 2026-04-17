package org.firstinspires.ftc.teamcode.old.PedroPathingAutos;

import org.firstinspires.ftc.teamcode.old.Config;
import org.firstinspires.ftc.teamcode.old.RTPAxon;

public class RobotControls {
    Config config;
    public double shootTime = 3.5;
    public double farShootTime = 6.0;
    public double farRevTime = 1.5; // extra time to let the shooter rev up
    public double farRevTime2 = 1.0;
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
        centerTurret();
    }

    public void enableShooterFar() {
        config.smartShooter.setVelocity(1925); // make it 1925 so it hits all the shots
        config.linearActuator.setPosition(0.25);
        centerTurret();
    }

    RTPAxon smartServo;

    private void centerTurret() {
        smartServo = new RTPAxon(config.turretServoLeft, config.axonServoEncoder);
        smartServo.forceResetTotalRotation();
        smartServo.setPidCoeffs(0.015, 0.0, 0.0001);
        smartServo.setRtp(true);
        smartServo.setTargetRotation(0.0);
    }

    public void updateSmartServo() {
        smartServo.update();
    }

    public void disableShooter() {
        config.smartShooter.setVelocity(0);
    }
}
