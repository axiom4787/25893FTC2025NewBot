package org.firstinspires.ftc.team28420.types;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public enum AprilTags {
    BLUE, GREEN, RED, WRONG;

    public static AprilTags getAprilTagFromId(AprilTagDetection detection) {
        if (detection.id == 20) {
            return BLUE;
        } else if (detection.id >= 21 && detection.id <= 23) {
            return GREEN;
        } else if (detection.id == 24) {
            return RED;
        } else {
            return WRONG;
        }
    }

    public static int aprilTagToGreen(AprilTagDetection detection) {
        return detection.id % 21;
    }
}
