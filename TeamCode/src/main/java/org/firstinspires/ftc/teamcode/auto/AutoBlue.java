package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "AutoBlue", group = "Auto")
public class AutoBlue extends OpMode {

    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private VisionPortal visionPortal;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = Blue.START_POSE;
    private final Pose aprilTagPose = Blue.APRILTAG_POSE;
    private final Pose scorePose = Blue.SCORE_POSE_NEAR;
    private final Pose align1Pose = Blue.ALIGN1_POSE;
    private final Pose pickup1Pose = Blue.PICKUP1_POSE;
    private final Pose align2Pose = Blue.ALIGN2_POSE;
    private final Pose pickup2Pose = Blue.PICKUP2_POSE;
    private final Pose align3Pose = Blue.ALIGN3_POSE;
    private final Pose pickup3Pose = Blue.PICKUP3_POSE;

    private Path aprilTagRead;
    private PathChain scorePreload, alignPickup1, grabPickup1, scorePickup1, alignPickup2, grabPickup2, scorePickup2, alignPickup3, grabPickup3, scorePickup3;

    public void buildPaths() {
        aprilTagRead = new Path(new BezierLine(startPose, aprilTagPose));
        aprilTagRead.setLinearHeadingInterpolation(startPose.getHeading(), aprilTagPose.getHeading());

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(aprilTagPose, scorePose))
                .setLinearHeadingInterpolation(aprilTagPose.getHeading(), scorePose.getHeading())
                .build();

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
                follower.followPath(aprilTagRead);
                mechController.setState(MechState.APRIL_TAG);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(scorePreload, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    mechController.setState(MechState.SHOOT_STATE); // Shoot preload
                    follower.followPath(alignPickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup1,true);
                    mechController.setState(MechState.INTAKE_STATE); //Grab 1
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    mechController.setState(MechState.SHOOT_STATE); // Shoot 1
                    follower.followPath(alignPickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup2,true);
                    mechController.setState(MechState.INTAKE_STATE); // Grab 2
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    mechController.setState(MechState.SHOOT_STATE); // Shoot 2
                    follower.followPath(alignPickup3,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup3,true);
                    mechController.setState(MechState.INTAKE_STATE); // Grab 3
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    mechController.setState(MechState.SHOOT_STATE); // Shoot 3
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        mechController.update();
        follower.update();
        autonomousPathUpdate();

        MechState state = mechController.getCurrentState();
        if (state == MechState.SHOOT_STATE || visionController.distanceSensor()) {
            follower.setMaxPower(0.0);
        } else if (state == MechState.INTAKE_STATE) {
            follower.setMaxPower(MechController.INTAKE_DRIVE_POWER);
        } else {
            follower.setMaxPower(MechController.FULL_DRIVE_POWER);
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Distance Sensor", visionController.distanceSensor());
        mechController.allTelemetry();
    }

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, telemetry);

        visionController = new VisionController(robot);
        visionController.initAprilTag();
        visionPortal = visionController.getVisionPortal();

        mechController = new MechController(robot, visionController);
        mechController.handleMechState(MechState.START);

        telemetry.addData("Status", "Initialized. Detecting April Tag....");
        telemetry.update();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        mechController.update();
        mechController.allTelemetry();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        visionPortal.stopStreaming();
        mechController.setLifter(0);
        mechController.setIndexer(MechController.INTAKE[0]);
    }
}
