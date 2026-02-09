package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimeLightCalculator {
    static Config config = new Config();
    Limelight3A limeLight;
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
        double base = -target.getTx() * 5f;
        return base;
    }

    public double calculateHood(LLResult target) {
        double base = -target.getTy() * 0.005f;
        return base;
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
