package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimeLightCalculator {
    static Config config = new Config();
    Limelight3A limeLight;
    public PID hoodPID = new PID(4e-5f, 0, 1e-5f);
    public PID turretPID = new PID(0f, 0f, 0f);
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
        return turretPID.calculate(target.getTx(), 0f);
    }

    public double calculateHood(LLResult target) {
        double base = hoodPID.calculate(-target.getTy(), 0f); // 0f is the center of the camera, so if we are off, we should make the motors move towards that
        return base * 2f;
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
