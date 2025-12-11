package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AprilTagLedManager {
    private final Limelight3A limelight;
    private final Servo rgbLightServo;

    private final double ZONE_MIN = 0.3;
    private final double ZONE_MAX = 1.0;
    private final double CAM_HEIGHT = 0.146812;
    private final double TAG_HEIGHT = 0.146812;
    private final double CAM_ANGLE = 0.0;

    public AprilTagLedManager(HardwareMap hardwareMap, String limelightName, String servoName) {
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        rgbLightServo = hardwareMap.get(Servo.class, servoName);
    }

    public void update() {
        boolean inZone = false;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleRad = Math.toRadians(CAM_ANGLE + ty);
            double distance = (TAG_HEIGHT - CAM_HEIGHT) / Math.tan(angleRad);
            distance = Math.abs(distance);
            inZone = (distance >= ZONE_MIN && distance <= ZONE_MAX);
        }

        double pulseWidth = inZone ? 1500 : 1000;
        double position = (pulseWidth - 500) / 2000.0;
        position = Math.max(0.0, Math.min(1.0, position));
        rgbLightServo.setPosition(position);
    }
}