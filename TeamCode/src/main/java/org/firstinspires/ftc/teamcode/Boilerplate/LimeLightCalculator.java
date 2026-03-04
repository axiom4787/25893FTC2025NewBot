package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

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

    public static class TagResult {
        public double tx, ty, ta;

        public TagResult(double tx, double ty, double ta) {
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
        }
    }

    public TagResult getTargetTag(int tagId) {
        LLResult llResult = limeLight.getLatestResult();
        if (!llResult.isValid()) return null;

        for (LLResultTypes.FiducialResult res : llResult.getFiducialResults()) {
            if (res.getFiducialId() == tagId) {
                return new TagResult(
                        res.getTargetXDegrees(),
                        res.getTargetYDegrees(),
                        res.getTargetArea()
                );
            }
        }

        return null;
    }

    public LLResult getTarget() {
        LLResult result = limeLight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

        if (result.isValid() && (fiducialResults.get(0).getFiducialId() == 24 | fiducialResults.get(0).getFiducialId() == 20)) {
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
