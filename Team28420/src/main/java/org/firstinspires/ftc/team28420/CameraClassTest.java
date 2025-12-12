package org.firstinspires.ftc.team28420;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team28420.module.Camera;
import org.firstinspires.ftc.team28420.util.Config;

@TeleOp(name = "camera class", group = "test")
public class CameraClassTest extends LinearOpMode {
    private Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        Config.Etc.telemetry = telemetry;

        waitForStart();

        while (opModeIsActive()) {
            camera.update();
            telemetry.addData("detections", Config.AprilTag.GREEN_POS);
        }
    }
}
