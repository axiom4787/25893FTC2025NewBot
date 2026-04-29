package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Hardware;

public class Insight {
    private final Limelight3A limelight;

    Telemetry telemetry;

    public Insight(Telemetry telemetry) {
        limelight = Hardware.getLimelight();
        limelight.setPollRateHz(100);
        this.telemetry = telemetry;
        limelight.pipelineSwitch(0);
    }

    public void updateBotPose(Follower follower) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return;

        Pose3D botpose = result.getBotpose();

        if (botpose == null || botpose.getPosition().x == 0 || botpose.getPosition().y == 0) return;

        double m_to_in = 39.37;

        double llX = botpose.getPosition().x;
        double llY = botpose.getPosition().y;
        double llH = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        double pedroX = llY * m_to_in + Globals.FIELD_WIDTH / 2;
        double pedroY = llX * m_to_in + Globals.FIELD_WIDTH / 2;
        double pedroH = llH - Math.PI / 2;

        follower.setX(pedroX); // sets heading to 0 just because

    }
}
