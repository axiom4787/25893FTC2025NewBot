package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoRed_Pedro", group = "Auto")
public class AutoRed_Pedro extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = Red.START_POSE;
    private final Pose scorePose = Red.SCORE_POSE;
    private final Pose align1Pose = Red.ALIGN1_POSE;
    private final Pose pickup1Pose = Red.PICKUP1_POSE;
    private final Pose align2Pose = Red.ALIGN2_POSE;
    private final Pose pickup2Pose = Red.PICKUP2_POSE;
    private final Pose align3Pose = Red.ALIGN3_POSE;
    private final Pose pickup3Pose = Red.PICKUP3_POSE;

    private Path scorePreload;
    private PathChain alignPickup1, grabPickup1, scorePickup1,
            alignPickup2, grabPickup2, scorePickup2,
            alignPickup3, grabPickup3, scorePickup3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        alignPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, align1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), align1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(align1Pose, pickup1Pose))
                .setLinearHeadingInterpolation(align1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        alignPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, align2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), align2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(align2Pose, pickup2Pose))
                .setLinearHeadingInterpolation(align2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        alignPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, align3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), align3Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(align3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(align3Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(alignPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(alignPickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(alignPickup3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    setPathState(-1); // finished
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** Called once at INIT button press */
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("AutoRed initialized");
        telemetry.update();
    }

    /** Called repeatedly after INIT, before START */
    @Override
    public void init_loop() {}

    /** Called once at START */
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** Called repeatedly after START */
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** Called once at STOP */
    @Override
    public void stop() {}
}
