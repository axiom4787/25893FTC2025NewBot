package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.util.Hardware;

public class Vision {
    private final Limelight3A limeLight;

    public Vision() {
        limeLight = Hardware.getLimelight();
    }

    public Pose getRobotPose() {
        return null;
    }
}
