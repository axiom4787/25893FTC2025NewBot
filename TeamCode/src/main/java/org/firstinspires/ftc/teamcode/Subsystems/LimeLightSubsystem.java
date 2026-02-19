package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.Boilerplate.LimeLightCalculator;
import org.firstinspires.ftc.teamcode.Boilerplate.PID;

public class LimeLightSubsystem {
    Config config = new Config();
    Limelight3A limeLight;
    public PID hoodPID = new PID(4e-5f, 0, 1e-5f, -1, 1);
    public PID turretPID = new PID(0.02, 0, 0.01, -1, 1);

    public LimeLightSubsystem(HardwareMap hardwareMap) {
        config.init(hardwareMap);
        limeLight = config.limeLight;
        limeLight.start();
    }

    public LLResult getTarget() {
        LLResult result = limeLight.getLatestResult();

        return result.isValid() ? result : null;
    }

    public double calculateTurret(LLResult target) {
        return turretPID.calculate(0f, target.getTx());
    }

    public double calculateHood(LLResult target) {
        return hoodPID.calculate(-target.getTy(), 0f) * 2f;
        // 0f is the center of the camera, so if we are off,
        // we should make the motors move towards that
    }
}
