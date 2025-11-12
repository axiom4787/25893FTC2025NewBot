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
    boolean buttonPressed = false;

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
        mechController.handleMechState(mechController.getCurrentState());

        // ----- APRIL_TAG handling -----
        if (gamepad2.a) {
            mechController.handleMechState(MechState.APRIL_TAG);
        }
        else if ((gamepad2.right_trigger > 0.2) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.INTAKE_STATE);
        } else if ((gamepad2.left_trigger > 0.2) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.SHOOT_STATE);
        } else if ((gamepad2.right_bumper) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.SHOOT_GREEN);
        } else if ((gamepad2.left_bumper) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.SHOOT_PURPLE);
        } else if ((gamepad2.b) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.HUMAN_STATE);
        } else if ((gamepad2.x) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.IDLE);
        }

        // Reset button press flag when no buttons (except A) are pressed
        if (!gamepad2.b && !gamepad2.x && gamepad2.left_trigger <= 0.2 && gamepad2.right_trigger <= 0.2) {
            buttonPressed = false;
        }

        // Vision Portal controls
        /*if (visionPortal != null) {
            if (gamepad1.dpad_down && !prevDpadDown) visionPortal.stopStreaming();
            else if (gamepad1.dpad_up && !prevDpadUp) visionPortal.resumeStreaming();
        }

        prevDpadDown = gamepad1.dpad_down;
        prevDpadUp = gamepad1.dpad_up;*/

        // Telemetry
        mechController.allTelemetry();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}