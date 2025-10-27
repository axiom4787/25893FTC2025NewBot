package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Movement;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

@TeleOp(name = "AprilTagTester", group = "AA_main")
public class AprilTagTester extends LinearOpMode {
    OpenCvCamera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTag aprilTag = new AprilTag(hardwareMap);
        AprilTagAimer aprilAimer = new AprilTagAimer(hardwareMap);
        Movement movement = new Movement(hardwareMap);
        GamepadEx gamePadOne = new GamepadEx(gamepad1);
        GamepadEx gamePadTwo = new GamepadEx(gamepad2);

        waitForStart();
        while (opModeIsActive()) {
            gamePadOne.readButtons();
            gamePadTwo.readButtons();

            movement.teleopTick(gamePadOne.getLeftX(),gamePadOne.getLeftY(),gamePadOne.getRightX());

            telemetry.addData("X:", "Scan obelisk apriltag");
            telemetry.addData("A", "Test april tag aimer with blue alliance apriltag");
            telemetry.addData("B", "Test april tag aimer with red alliance apriltag");
            telemetry.update();

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.A)) {
                aprilTag.setGoalTagID(20);
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();
                aprilAimer.startTurnToAprilTag(bearing);

                while (!aprilAimer.updateTurn()) {
                    telemetry.addData("Turning towards angle", bearing);
                    telemetry.update();
                    sleep(10);
                }

                telemetry.addData("Finished", "locking on to apriltag");
                telemetry.update();
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.B)) {
                aprilTag.setGoalTagID(24);
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();
                aprilAimer.startTurnToAprilTag(bearing);

                while (!aprilAimer.updateTurn()) {
                    telemetry.addData("Turning towards angle", bearing);
                    telemetry.update();
                    sleep(10);
                }

                telemetry.addData("Finished", "locking on to apriltag");
                telemetry.update();
            }

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.X)) {
                aprilTag.scanObeliskTag();
                telemetry.addData("This is probably only for auto,", "as we can just memorize the 3 possible patterns for teleop");
                telemetry.addData("Obelisk apriltag ID: ", aprilTag.getObeliskId());
                telemetry.update();
            }
        }
        //vibe code stuff for the camera stream
        // Get webcam from config
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");

        // Get the viewport ID to display on Driver Hub / Dashboard
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "webcam", "id", hardwareMap.appContext.getPackageName());

        // Create a webcam instance with live preview ID
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Simple pipeline that just returns the camera feed unchanged
        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Imgproc.putText(input, "Preview Active", new org.opencv.core.Point(10, 30),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 1, new org.opencv.core.Scalar(255, 255, 255), 2);
                return input;
            }
        });

        // Open camera asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error: ", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Press INIT, then open Camera Stream from menu");
        telemetry.update();

        waitForStart();

        // Stream continues while OpMode runs
        while (opModeIsActive()) {
            telemetry.addData("Status", "Streaming...");
            telemetry.update();
            sleep(50);
        }

        camera.stopStreaming();
    }
    }
