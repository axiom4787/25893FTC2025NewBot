package org.firstinspires.ftc.teamcode.testOpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp(name = "test vision pose")
public class TestTheRelocalization extends LinearOpMode {
    public Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(71, 71, 0));

        Context.init(this);

        Vision vision = new Vision();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            vision.limelight.updateRobotOrientation(Math.toDegrees(
                    Context.follower.getHeading() + Math.PI / 2
            ));

            Pose visionPose = vision.getPose();

            if (visionPose == null) continue;

            double x = visionPose.getX();
            double y = visionPose.getY();
            double h = visionPose.getHeading();

            PanelsDrawing.drawDebug(follower);

            telemetry.addLine("vision coords");
            telemetry.addData("vision x", x);
            telemetry.addData("vision y", y);
            telemetry.addData("vision h", h);
            telemetry.addData("odo", follower.getPose().toString());
            telemetry.update();
        }
    }
}
