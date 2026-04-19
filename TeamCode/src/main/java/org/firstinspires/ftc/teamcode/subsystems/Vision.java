package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Hardware;

import java.util.Arrays;
import java.util.OptionalDouble;

public class Vision {
    private final Limelight3A limeLight;

    // Params
    double trustThreshold = 0.2; // meters; I want to have vision be within at least 20cm

    public Vision() {
        limeLight = Hardware.getLimelight();
    }

    public void start() {
        limeLight.pipelineSwitch(0);
    }

    private Pose3D botpose;
    double trust = 0.0; // How much to mix the limelight pose with the other pose (0 to 1)
    public void update(double gyroYawDegrees) {
        limeLight.updateRobotOrientation(gyroYawDegrees);
        LLResult result = limeLight.getLatestResult();

        if (result == null) return;
        botpose = result.getBotpose_MT2();

        if (botpose == null) return; // Apparently this can technically be null if the limelight can't solve the transform
        if (!botpose.getPosition().equals(new Position())) {
            double[] sx = result.getStddevMt2();
            trust = Math.max(0, 1.0 - (Math.max(sx[0], sx[1]) / trustThreshold));
        } else {
            trust = 0.0;
        }
    }

    // Get the last updated robot pose
    private Pose getRobotPose() {
        double m_to_in = 39.37; // 1m = 39.37 in
        return new Pose(botpose.getPosition().x * m_to_in,
                botpose.getPosition().y * m_to_in,
                Math.toRadians(botpose.getOrientation().getYaw()));
    }

    public Follower updatePose(Follower follower) {
        if (trust <= 0.0) return null;
        if (botpose == null) return null;
        if (botpose.getPosition().equals(new Position())) return null;

        Pose currentPose = follower.getPose();
        Pose visionPose = getRobotPose();

        // Mix X and Y
        double mixedX = (currentPose.getX() * (1 - trust)) + (visionPose.getX() * trust);
        double mixedY = (currentPose.getY() * (1 - trust)) + (visionPose.getY() * trust);

        // But not heading
        double currentHeading = currentPose.getHeading();

        follower.setPose(new Pose(mixedX, mixedY, currentHeading));

        return follower;
    }
}
