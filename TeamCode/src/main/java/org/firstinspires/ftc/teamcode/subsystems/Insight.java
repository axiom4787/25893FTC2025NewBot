package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Hardware;

public class Insight {
    private final Limelight3A limelight;

    private static final double trustThreshold = 0.2; // 0.2 meters

    public double trust = 0.0;

    public Insight() {
        limelight = Hardware.getLimelight();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
    }

    public void updateBotPose(Follower follower) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return; // new Pose();

        Pose3D botpose = result.getBotpose();

        if (botpose == null || botpose.getPosition().x == 0 || botpose.getPosition().y == 0) return; // new Pose();

        double[] sx = result.getStddevMt1();
        trust = Math.max(0, 1.0 - (Math.max(sx[0], sx[1]) / trustThreshold));

        if (trust < 0.85) return;

        double m_to_in = 39.37;

        double llX = botpose.getPosition().x;
        double llY = botpose.getPosition().y;
        double llH = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        double pedroX = llY *  m_to_in + Globals.FIELD_WIDTH / 2;
        double pedroY = llX * -m_to_in + Globals.FIELD_WIDTH / 2;
        double pedroH = llH - Math.PI / 2;

        follower.setPose(new Pose(pedroX, pedroY, pedroH));
//        return new Pose(pedroX, pedroY, pedroH);
    }
}
