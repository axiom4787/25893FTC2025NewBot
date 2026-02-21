package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimeLightCalculator {
    static Config config = new Config();
    Limelight3A limeLight;
    public PID hoodPID = new PID(0.0005, 0, 1e-5f, -1, 1);
    public PID turretPID = new PID(0.02, 0, 0.01, -1, 1);
    public LimeLightCalculator(HardwareMap hardwareMap) {
        config.init(hardwareMap);
        limeLight = config.limeLight;
        limeLight.start();
    }
    public LLResult getTarget() {
        LLResult result = limeLight.getLatestResult();
        if (result.isValid()) {
            return result;
        } else {
            return null;
        }
    }

    public double calculateTurret(LLResult target) {
        return turretPID.calculate(0f, target.getTx());
    }

    public double calculateHood(LLResult target) {
        return hoodPID.calculate(-target.getTy(), 0f) * 2f; // 0f is the center of the camera, so if we are off, we should make the motors move towards that
    }

    public enum LogWhat {
        TX, TY, TA
    }
    public double log(LLResult target, LogWhat what) {
        switch (what) {
            case TX:
                return target.getTx();
            case TY:
                return target.getTy();
            case TA:
                return target.getTa();
            default:
                return 0.0;
        }
    }
}
