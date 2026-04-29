package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Hardware;

import java.util.Arrays;
import java.util.OptionalDouble;

public class Vision {
    private final Limelight3A limeLight;
    public double gottenHeading = 67;

    // Params
    double trustThreshold = 0.2; // meters; I want to have vision be within at least 10cm

    Telemetry telemetry;
    public Vision(Telemetry telemetry) {
        limeLight = Hardware.getLimelight();
        limeLight.setPollRateHz(100);
        this.telemetry = telemetry;
        start();
    }

    public void start() {
        limeLight.pipelineSwitch(0);
    }

    private Pose3D botpose;
    double trust = 0.0; // How much to mix the limelight pose with the other pose (0 to 1)

    public void update() {
        LLResult result = limeLight.getLatestResult();

        if (result == null) return;
        botpose = result.getBotpose();

        if (botpose == null) return; // Apparently this can technically be null if the limelight can't solve the transform
        if (!botpose.getPosition().equals(new Position())) {
            double[] sx = result.getStddevMt1();
            trust = Math.max(0, 1.0 - (Math.max(sx[0], sx[1]) / trustThreshold));
        } else {
            trust = 0.0;
        }
    }

    // Get the last updated robot pose
    public Pose getRobotPose(Follower follower) {
        double m_to_in = 39.37; // 1m = 39.37 in
        // LL:
        //  negative y -> blue goal
        //  negative x -> obelisk
        //  positive y -> red goal
        //  positive x -> far zone
        //  0 deg      -> facing far zone
        //  90 deg     -> facing red goal
        //  heading increases ccw

        // pedro:
        //  0y    -> far zone
        //  0x    -> blue goal
        //  max y -> obelisk
        //  max x -> red goal
        //  0 deg -> facing red goal
        //  heading increases ccw

        double llX = botpose.getPosition().x;
        double llY = botpose.getPosition().y;
        double llH = botpose.getOrientation().getYaw();

        double pedroX = llY *  m_to_in + Globals.FIELD_WIDTH / 2; // good?
        double pedroY = llX * -m_to_in + Globals.FIELD_WIDTH / 2; // good?
        double pedroH = Math.toRadians(llH - 90); // good?

        return new Pose(pedroX, pedroY, follower.getHeading());
    }

    public void updatePose(Follower follower) {
//        telemetry.addData("LOOK HERE --->", botpose);
//        telemetry.addData("Trust -->>", trust);
//        telemetry.addData("ASDF", h);

        if (trust <= 0.0) return;
        if (botpose == null) return;
        Position testPose = botpose.getPosition();
        if (testPose.x == 0f && testPose.y == 0f) return;

        Pose visionPose = getRobotPose(follower);
//        telemetry.addData("LOOK AT ME, TOO!!! -->", visionPose);

        // Mix X, Y, and heading
//        double mixedX = (currentPose.getX() * (1 - trust)) + (visionPose.getX() * trust);
//        double mixedY = (currentPose.getY() * (1 - trust)) + (visionPose.getY() * trust);
//        double mixedH = (currentPose.getHeading() * (1 - trust) + (visionPose.getHeading() * trust));

        if (trust < 0.85) return;
        // Do not set pose from vision if we don't trust the LL

        follower.setPose(visionPose);

//        double mixedX = visionPose.getX();
//        double mixedY = visionPose.getY();
//
////        follower.setX(mixedX);
////        follower.setY(mixedY);
//
//        gottenHeading = follower.getHeading();
//
//        follower.setPose(follower.getPose().withX(mixedX).withY(mixedY));
//
//        follower.setHeading(gottenHeading);

//        follower.setHeading(follower.getHeading());

        // follower.setPose(new Pose(mixedX, mixedY, mixedH));
    }
}
