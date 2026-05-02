package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Globals;

public class Vision {
    private final Limelight3A limelight;

    private static final double trustThreshold = 0.2; // 0.2 meters

    public double trust = 0.0;

    public ElapsedTime timer = new ElapsedTime();

    public Vision() {
        limelight = Context.getLimelight();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        timer.reset();
    }

    public Pose getPose() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return null;

        Pose3D botpose = result.getBotpose();

        if (botpose == null || botpose.getPosition().x == 0 || botpose.getPosition().y == 0) return null;

        double[] sx = result.getStddevMt1();
        trust = Math.max(0, 1.0 - (Math.max(sx[0], sx[1]) / trustThreshold));

        if (trust < 0.85) return null;

        return convertLLtoPedro(botpose);
    }

    public void update() {
        boolean isOdoMovingXY = Context.follower.getVelocity().getMagnitude() > 4;
        boolean isOdoMovingHeading = Math.abs(Context.follower.getVelocity().getTheta()) > Math.toRadians(20);
        boolean isOdoMoving = isOdoMovingXY || isOdoMovingHeading;

        if (timer.milliseconds() > 500 && !isOdoMoving) {
            timer.reset();

            Pose visionPose = getPose();
            if (visionPose != null) Context.follower.setPose(visionPose);
        }
    }

    private Pose convertLLtoPedro(Pose3D llPose) {
        double m_to_in = 39.37;

        double llX = llPose.getPosition().x;
        double llY = llPose.getPosition().y;
        double llH = llPose.getOrientation().getYaw(AngleUnit.RADIANS);

        double pedroX = llY *  m_to_in + Globals.FIELD_WIDTH / 2;
        double pedroY = llX * -m_to_in + Globals.FIELD_WIDTH / 2;
        double pedroH = llH - Math.PI / 2;

        return new Pose(pedroX, pedroY, pedroH);
    }

    // Pose conversion

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
}
