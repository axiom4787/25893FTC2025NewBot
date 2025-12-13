package org.firstinspires.ftc.team28420;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team28420.module.Camera;
import org.firstinspires.ftc.team28420.util.Config;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name = "camera class", group = "test")
public class CameraClassTest extends LinearOpMode {
    private Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        Config.Etc.telemetry = telemetry;

        waitForStart();

        while (opModeIsActive()) {
            AprilTagDetection detection = camera.getBinTag().orElse(null);
            if (detection == null) {
                continue;
            }

//            AprilTagPoseFtc pose = detection.ftcPose;
//
//            if (pose == null) {
//                continue;
//            }

//            telemetry.addData("x", pose.x);
//            telemetry.addData("y", pose.y);
//            telemetry.addData("z", pose.z);
//            telemetry.addData("range", pose.range);
//            telemetry.addData("pitch", pose.pitch);


            telemetry.addData("x", detection.robotPose.getPosition().x);
            telemetry.addData("y", detection.robotPose.getPosition().y);
            telemetry.addData("z", detection.robotPose.getPosition().z);
            telemetry.update();
        }
    }
}
