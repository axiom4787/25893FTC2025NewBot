package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
/**
 * Variables to store the position and orientation of the camera on the robot. Setting these
 * values requires a definition of the axes of the camera and robot:
 *
 * Camera axes:
 * Origin location: Center of the lens
 * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
 *
 * Robot axes (this is typical, but you can define this however you want):
 * Origin location: Center of the robot at field height
 * Axes orientation: +x right, +y forward, +z upward
 *
 * Position:
 * If all values are zero (no translation), that implies the camera is at the center of the
 * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
 * inches above the ground - you would need to set the position to (-5, 7, 12).
 *
 * Orientation:
 * If all values are zero (no rotation), that implies the camera is pointing straight up. In
 * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
 * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
 * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
 * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
 */
@TeleOp(name = "AprilTag Blue Localization", group = "PedroPathing")
public class TeleopAprilTag extends OpMode {

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0.5, 7.6, 9.7, 0);//-0.5, -7.6, -9.7
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Follower follower;
    private boolean following = false;

    private final Pose startPose = new Pose(80, 38, Math.toRadians(90));
    private final Pose TARGET_LOCATION = new Pose(80, 62, Math.toRadians(54));
    private final Pose endPose = new Pose(80, 72, Math.toRadians(90));


    @Override
    public void init() {
        initAprilTag();
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

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private Pose getRobotPoseFromCamera() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (!detection.metadata.name.contains("Obelisk")) {
                    double x = ((detection.robotPose.getPosition().y)+72);
                    double y =((-detection.robotPose.getPosition().x)+72);
                    double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    return new Pose(x, y, Math.toRadians(heading-76));//Remove 76
                }
            }
        }
        return null;
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
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)-76));//Remove 76
                }
            } else {
                telemetry.addLine(String.format(Locale.US,"\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.US,"Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }
}