package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "TeleopIndexerTest", group = "Teleop")
public class TeleopIndexerTest extends OpMode {

    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private VisionPortal visionPortal;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    boolean buttonPressed = false;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, telemetry);

        visionController = new VisionController(robot);
        visionController.initAprilTag();
        visionPortal = visionController.getVisionPortal();

        mechController = new MechController(robot, visionController);
        mechController.handleMechState(MechState.START);

        telemetry.addData("Status", "Initialized. Detecting April Tag....");
        telemetry.update();
    }

    @Override

    public void init_loop() {
        mechController.update();
        mechController.allTelemetry();
    }

    @Override
    public void loop() {
        mechController.update(); // Keeps running states till IDLE

        if (mechController.getCurrentState() == MechState.INTAKE_STATE) {
            int nextIndex = mechController.getEmptyIndex();
            if (nextIndex != -1) {
                mechController.setIndexerIntake(MechController.INTAKE[nextIndex]);
            }
        }

        // ----- APRIL_TAG handling -----
        if ((gamepad2.a) && !buttonPressed) {
            buttonPressed = true;
            mechController.setState(MechState.APRIL_TAG);
        }
        else if ((gamepad2.left_trigger > 0.2) && !buttonPressed) {
            buttonPressed = true;
            mechController.setState(MechState.INTAKE_STATE);
        } else if ((gamepad2.right_trigger > 0.2) && !buttonPressed) {
            buttonPressed = true;
            mechController.setState(MechState.SHOOT_STATE);
        } else if ((gamepad2.left_bumper) && !buttonPressed) {
            buttonPressed = true;
            mechController.setState(MechState.SHOOT_GREEN);
        } else if ((gamepad2.right_bumper) && !buttonPressed) {
            buttonPressed = true;
            mechController.setState(MechState.SHOOT_PURPLE);
        } else if ((gamepad2.b) && !buttonPressed) {
            buttonPressed = true;
            mechController.setState(MechState.HUMAN_STATE);
        } else if ((gamepad2.x) && !buttonPressed) {
            buttonPressed = true;
            mechController.setState(MechState.IDLE);
        } else if ((gamepad2.dpad_up) && !buttonPressed) {
            buttonPressed = true;
            mechController.tagPattern = new int[]{21, 2, 1, 1}; // ID 21: GPP
        } else if ((gamepad2.dpad_right) && !buttonPressed) {
            buttonPressed = true;
            mechController.tagPattern = new int[]{22, 1, 2, 1}; // ID 22: PGP
        } else if ((gamepad2.dpad_down) && !buttonPressed) {
            buttonPressed = true;
            mechController.tagPattern = new int[]{23, 1, 1, 2}; // ID 23: PPG
        }

        // Reset button press flag when no buttons (except A) are pressed
        boolean noButtons =
                !gamepad2.b &&
                        !gamepad2.x &&
                        gamepad2.left_trigger <= 0.2 &&
                        gamepad2.right_trigger <= 0.2 &&
                        !gamepad2.left_bumper &&
                        !gamepad2.right_bumper &&
                        !gamepad2.a &&
                        !gamepad2.dpad_up &&
                        !gamepad2.dpad_right &&
                        !gamepad2.dpad_down;

        if (noButtons) {
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