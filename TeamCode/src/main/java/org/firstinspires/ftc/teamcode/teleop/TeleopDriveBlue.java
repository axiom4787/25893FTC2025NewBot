package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleopBlue_Pedro", group = "Teleop")
public class TeleopDriveBlue extends OpMode {
    private Follower follower;
    private final Pose startingPose = Blue.SCORE_POSE;
    private final Pose scorePose = Blue.SCORE_POSE;
    private final Pose endgamePose = Blue.ENDGAME_POSE;
    private final Pose gateStartPose = Blue.GATE_START_POSE;
    private final Pose gateEndPose = Blue.GATE_END_POSE;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    boolean buttonPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new RobotHardware(hardwareMap, telemetry);

        visionController = new VisionController(robot);
        visionController.initAprilTag();

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
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        mechController.update(); // Keeps running states till IDLE

        MechState state = mechController.getCurrentState();
        if (state == MechState.SHOOT_STATE || state == MechState.SHOOT_PURPLE || state == MechState.SHOOT_GREEN || visionController.distanceSensor()) {
            follower.setMaxPower(0.0);
        } else if (state == MechState.INTAKE_STATE) {
            follower.setMaxPower(MechController.INTAKE_DRIVE_POWER);
        } else {
            follower.setMaxPower(MechController.FULL_DRIVE_POWER);
        }

        if ((gamepad2.left_trigger > 0.2) && !buttonPressed) {
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

        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
        //Shooting Pose
        if (gamepad1.right_trigger > 0.1 && !automatedDrive) {
            PathChain shootingPath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.getHeading(), 0.8))
                    .build();
            follower.followPath(shootingPath);
            automatedDrive = true;
        }

        //Gate Pose
        if (gamepad1.left_trigger > 0.1 && !automatedDrive) {
            PathChain gatePath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, gateStartPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateStartPose.getHeading(), 0.8))
                    .addPath(new Path(new BezierLine(follower::getPose, gateEndPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gateEndPose.getHeading(), 0.8))
                    .build();
            follower.followPath(gatePath);
            automatedDrive = true;
        }

        //Endgame Pose
        if (gamepad1.xWasPressed()) {
            PathChain endgamePath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, endgamePose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, endgamePose.getHeading(), 0.8))
                    .build();
            follower.followPath(endgamePath);
            automatedDrive = true;
        }

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

        /*Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }*/

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.aWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
