package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "TeleopMech", group = "Teleop")
public class TeleopMech extends OpMode {

    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, telemetry);
        mechController = new MechController(robot);
        visionController = new VisionController(robot);

        visionController.initAprilTag();
        mechController.handleMechState(MechState.IDLE);

        aprilTag = visionController.getAprilTag();
        visionPortal = visionController.getVisionPortal();

        telemetry.addData("Status", "Initialized. Press START.");
        telemetry.update();
    }

    @Override
    public void loop() {
        int tagID = visionController.findTagID(aprilTag);

        switch (tagID) {
            case 1:
                mechController.setIndexer(-120);
                break;
            case 2:
                mechController.setIndexer(0);
                break;
            case 3:
                mechController.setIndexer(120);
                break;
            default:
                // keep servo still if no tag
                break;
        }

        if (visionPortal != null) {
            if (gamepad1.dpad_down && !prevDpadDown) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up && !prevDpadUp) {
                visionPortal.resumeStreaming();
            }
        }

        prevDpadDown = gamepad1.dpad_down;
        prevDpadUp = gamepad1.dpad_up;
        mechController.allTelemetry();
        telemetry.addData("Detected Tag", tagID);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}