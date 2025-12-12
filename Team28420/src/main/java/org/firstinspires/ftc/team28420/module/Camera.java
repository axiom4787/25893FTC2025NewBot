package org.firstinspires.ftc.team28420.module;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team28420.types.AprilTags;
import org.firstinspires.ftc.team28420.util.Config;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Camera {
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    public Camera(WebcamName webcam) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcam, aprilTag);
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public void update() {
        for (AprilTagDetection detection : getDetections()) {
            switch (AprilTags.getAprilTagFromId(detection)) {
                case RED:
                    Config.Etc.telemetry.addData("rd range", detection.ftcPose);
                    break;
                case GREEN:
                    Config.Etc.telemetry.addData("gr range", detection.ftcPose.range);
                    setGreenPos(AprilTags.aprilTagToGreen(detection));
                    break;
                case BLUE:
                    Config.Etc.telemetry.addData("bl range", detection.ftcPose.range);
                    break;
                case WRONG:
                    Config.Etc.telemetry.addLine("wrong id");
                    break;
            }
        }
    }

    public void setGreenPos(int pos) {
        if (Config.AprilTag.GREEN_POS == -1)
            Config.AprilTag.GREEN_POS = pos;
    }
}
