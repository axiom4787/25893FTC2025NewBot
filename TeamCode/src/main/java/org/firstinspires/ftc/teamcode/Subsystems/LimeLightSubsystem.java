package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Boilerplate.PID;
import org.firstinspires.ftc.teamcode.Hardware.CachingHardware;

public class LimeLightSubsystem {
    private final Limelight3A limeLight;
    public final PID hoodPID = new PID(4e-5f, 0, 1e-5f, -1, 1);
    public final PID turretPID = new PID(0.02, 0, 0.01, -1, 1);

    public LimeLightSubsystem() {
        limeLight = CachingHardware.getLimelight();
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
