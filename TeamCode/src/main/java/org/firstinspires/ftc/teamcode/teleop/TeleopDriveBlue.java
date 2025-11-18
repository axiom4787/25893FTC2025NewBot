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
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleopBlue", group = "Teleop")
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
    private VisionPortal visionPortal;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
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
        visionPortal = visionController.getVisionPortal();

        mechController = new MechController(robot, visionController);
        mechController.handleMechState(MechState.APRIL_TAG);

        telemetry.addData("Status", "Initialized. Detecting April Tag....");
        telemetry.update();
    }

    @Override

    public void init_loop() {
        mechController.handleMechState(mechController.getCurrentState());
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
        //Call this once per loop
        follower.update();
        telemetryM.update();

        mechController.handleMechState(mechController.getCurrentState()); // Keeps running states till IDLE

        if (gamepad2.aWasPressed()) {
            mechController.handleMechState(MechState.APRIL_TAG);
        } else if ((gamepad2.left_trigger > 0.2) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.INTAKE_STATE);
        } else if ((gamepad2.right_trigger > 0.2) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.SHOOT_STATE);
        } else if ((gamepad2.left_bumper) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.SHOOT_GREEN);
        } else if ((gamepad2.right_bumper) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.SHOOT_PURPLE);
        } else if ((gamepad2.b) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.HUMAN_STATE);
        } else if ((gamepad2.x) && !buttonPressed) {
            buttonPressed = true;
            mechController.handleMechState(MechState.IDLE);
        }
        boolean noButtons =
                !gamepad2.b &&
                        !gamepad2.x &&
                        gamepad2.left_trigger <= 0.2 &&
                        gamepad2.right_trigger <= 0.2 &&
                        !gamepad2.left_bumper &&
                        !gamepad2.right_bumper &&
                        !gamepad2.a;

        if (noButtons) {
            buttonPressed = false;
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }
        //Shooting Pose
        if (gamepad1.left_trigger > 0.1) {
            PathChain shootingPath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.getHeading(), 0.8))
                    .build();
            follower.followPath(shootingPath);
            automatedDrive = true;
        }

        //Gate Pose
        if (gamepad1.right_trigger > 0.1) {
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
            slowModeMultiplier = Math.min(1.5, slowModeMultiplier + 0.25);
        }

        //Optional way to change slow mode strength
        if (gamepad1.aWasPressed()) {
            slowModeMultiplier = Math.max(0.1, slowModeMultiplier - 0.25);
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        mechController.allTelemetry();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
