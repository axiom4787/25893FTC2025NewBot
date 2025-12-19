package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.theKeep.TheKeepAuto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class PedroPathing {

    public boolean automatedDrive;
    public Follower follower;
    public Supplier<PathChain> turnToGoal;

    // Sets up a bunch of positions that the bot uses to create the paths - Jason
    public static Pose startPose;
    private final Pose redAlliance1 = new Pose(57,9,Math.toRadians(90));
    private final Pose redAlliance2 = new Pose(63,135,Math.toRadians(-90));
    private final Pose blueAlliance1 = new Pose(87,9,Math.toRadians(90));
    private final Pose blueAlliance2 = new Pose(81,135,Math.toRadians(-90));
    private final Pose redGoalPose = new Pose(144, 144);
    private final Pose blueGoalPose = new Pose(0,144);

    // A method that sets up the follower for pedro pathing - Jason
    public void initFollower(HardwareMap hardwareMap) {

        // Sets up the follower based on the given position and tuned values from the constants class - Jason
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        // Decides which goal the PathChain turnToGoal should face based on the chosen alliance - Jason
        if (TheKeepAuto.alliance == TheKeepAuto.Alliance.RED) {
            turnToGoal = () -> follower.pathBuilder()
                    .addPath(new BezierPoint(follower::getPose))
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(redGoalPose))
                    .build();
        } else {
            turnToGoal = () -> follower.pathBuilder()
                    .addPath(new BezierPoint(follower::getPose))
                    .setHeadingInterpolation(HeadingInterpolator.facingPoint(blueGoalPose))
                    .build();
        }
    }

    // A method that simply sets the start position based on what was the chosen alliance and position - Jason
    public void setStartPose(boolean atPromptEnd) {
        if (atPromptEnd) {
            if (TheKeepAuto.alliance == TheKeepAuto.Alliance.RED) {
                if (TheKeepAuto.startLocation == 1) {
                    startPose = redAlliance1;
                } else startPose = redAlliance2;
            } else {
                if (TheKeepAuto.startLocation == 1) {
                    startPose = blueAlliance1;
                } else startPose = blueAlliance2;
            }
        } else {
            startPose = follower.getPose();
        }

    }

}