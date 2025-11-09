package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "TeleopAprilTag", group = "Teleop")
public class TeleopAprilTag extends OpMode {

    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Follower follower;
    private boolean following = false;

    private final Pose startPose = new Pose(80, 38, Math.toRadians(90));
    private final Pose TARGET_LOCATION = new Pose(80, 62, Math.toRadians(54));
    private final Pose endPose = new Pose(80, 72, Math.toRadians(90));


    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, telemetry);
        mechController = new MechController(robot);
        visionController = new VisionController(robot);

        visionController.initAprilTag();
        mechController.handleMechState(MechState.IDLE);

        aprilTag = visionController.getAprilTag();
        visionPortal = visionController.getVisionPortal();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.addData("position", follower.getPose());
        telemetryAprilTag();
        telemetry.update();
    }

    @Override
    public void start() {
        visionPortal.resumeStreaming();
    }
    @Override
    public void loop() {
        follower.update();

        if (!following && gamepad1.a) {
            PathChain targetPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                    .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.getHeading())
                    .build();
            follower.followPath(targetPath);
            following = true;
        }

        if (!following && gamepad1.x) {
            //visionPortal.resumeStreaming();
            Pose updatedPose = getRobotPoseFromCamera();
            if (updatedPose != null) {
                follower.setPose(updatedPose);
            }
            //visionPortal.stopStreaming();
            following = true;
        }

        if (!following && gamepad1.b) {
            PathChain correctionPath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, endPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endPose.getHeading(), 0.8))
                    .build();
            follower.followPath(correctionPath);
            following = true;
        }

        if (following && !follower.isBusy()) {
            following = false;
        }

        telemetry.addData("position", follower.getPose());
        telemetryAprilTag();
        telemetry.update();
    }

    private Pose getRobotPoseFromCamera() {
        if (aprilTag == null || aprilTag.getDetections().isEmpty()) return null;
        else {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (!detection.metadata.name.contains("Obelisk")) {
                        double x = ((detection.robotPose.getPosition().y) + 72);
                        double y = ((-detection.robotPose.getPosition().x) + 72);
                        double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                        return new Pose(x, y, Math.toRadians(heading));
                    }
                }
            }
            return null;
        }
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) %s", detection.id, detection.metadata.name));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format(Locale.US,"XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format(Locale.US,"PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.addLine(String.format(Locale.US,"XYH %6.1f %6.1f %6.1f  (inch inch deg)",
                            ((detection.robotPose.getPosition().y)+72),
                            ((-detection.robotPose.getPosition().x)+72),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.US,"Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }
}