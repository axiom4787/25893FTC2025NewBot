package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Boilerplate.LimeLightCalculator;
import org.firstinspires.ftc.teamcode.Boilerplate.PID;

public class LimeLightSubsystem {
    private final Limelight3A limeLight;
    public final PID hoodPID = new PID(4e-5f, 0, 1e-5f, -1, 1);
    public final PID turretPID = new PID(0.02, 0, 0.01, -1, 1);

    public LimeLightSubsystem(Limelight3A limeLight) {
        this.limeLight = limeLight;
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

    public LimeLightCalculator.TagResult getTargetTag(int tagId) {
        LLResult llResult = limeLight.getLatestResult();
        if (!llResult.isValid()) return null;

        for (LLResultTypes.FiducialResult res : llResult.getFiducialResults()) {
            if (res.getFiducialId() == tagId) {
                return new LimeLightCalculator.TagResult(
                        res.getTargetXDegrees(),
                        res.getTargetYDegrees(),
                        res.getTargetArea()
                );
            }
        }

        return null;
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
