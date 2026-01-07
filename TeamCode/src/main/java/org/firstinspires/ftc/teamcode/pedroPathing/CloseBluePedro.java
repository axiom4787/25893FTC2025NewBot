package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.GoalTagLimelight;
import org.firstinspires.ftc.teamcode.Shooter;

import java.nio.file.Paths;

@Disabled
@Autonomous(name = "CloseBluePedro", group = "Autonomous")
@Configurable // Panels
public class CloseBluePedro extends LinearOpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)

    Chassis ch;
    Shooter shooterLeft;
    Shooter shooterRight;
    Servo launchFlapLeft;
    Servo launchFlapRight;
    Shooter collectorBack;
    Shooter collectorFront;
    Servo flipper;
    GoalTagLimelight limelight;

    private int startDelay = 0;
    private int teamID;
    private boolean testingMode = false;
    private boolean shooting = false;
    ;
    ElapsedTime timer = new ElapsedTime();
    public PathChain MOVETOLAUNCH;
    public PathChain MOVETOCOLLECT;
    public PathChain COLLECT1;
    public PathChain COLLECT2;
    public PathChain COLLECT3;
    public PathChain MOVETOLAUNCH2;
    public PathChain ENDOFFLINE;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(26.359, 130.756, Math.toRadians(325)));

        buildPaths();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        ch = new Chassis(hardwareMap);

        collectorFront = new Shooter(hardwareMap, "collectorFront", false);

        collectorBack = new Shooter(hardwareMap, "collectorBack", false);

        flipper = hardwareMap.get(Servo.class, "flipper");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);

        shooterRight = new Shooter(hardwareMap, "shooterRight", false);

        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        launchFlapRight = hardwareMap.get(Servo.class, "launchFlapRight");

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap, telemetry);

        GlobalStorage.setPattern(null);
        GlobalStorage.setAlliance(-1);

        do {
            limelight.readObelisk(telemetry);
            GlobalStorage.setPattern(limelight.getObelisk());

            telemetry.addData("Pattern", limelight.getObelisk());
            telemetry.addData("Is Tag Recent", limelight.seeObelisk);
            telemetry.addData("team ID", teamID);
            telemetry.addData("Testing Mode", testingMode);
            telemetry.addLine("Press b for red, x for blue, y adds delay, a removes delay");
            telemetry.addData("Start Delay", startDelay);
            telemetry.addData("collectorPower", Shooter.collectorPower);
            telemetry.update();

            if (gamepad1.bWasPressed()) {
                //goalTag.targetAprilTagID = 24;
                teamID = 24;
                GlobalStorage.setAlliance(24);
            } else if (gamepad1.xWasPressed()) {
                //goalTag.targetAprilTagID = 20;
                teamID = 20;
                GlobalStorage.setAlliance(20);
            } else if (gamepad1.yWasPressed()) {
                startDelay += 2;
            } else if (gamepad1.aWasPressed()) {
                startDelay -= 1;
            } else if (gamepad1.leftStickButtonWasPressed()) {
                testingMode = true;
            }

        } while (opModeInInit());

        while (opModeIsActive()) {
            //sleep(1000*startDelay);

            limelight.setTeam();

            collectorBack.setPower(Shooter.collectorPower);
            collectorFront.setPower(Shooter.collectorPower);

            launchFlapLeft.setPosition(0.3);
            launchFlapRight.setPosition(0.4);

            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
    }

    public void buildPaths() {
        MOVETOLAUNCH = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(26.359, 130.756), new Pose(59.470, 84.400))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(135))
                .build();

        MOVETOCOLLECT = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.470, 84.400), new Pose(42.460, 84.271))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        COLLECT1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.460, 84.271), new Pose(34.409, 84.271))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        COLLECT2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(34.409, 84.271), new Pose(29.735, 84.400))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        COLLECT3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(29.735, 84.400), new Pose(24.671, 84.271))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        MOVETOLAUNCH2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(24.671, 84.271), new Pose(59.600, 84.300))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        ENDOFFLINE = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.600, 84.300), new Pose(29.865, 84.790))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                .build();
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(MOVETOLAUNCH, Shooter.maxPower, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    Shooter.fireVolleySorted(limelight, telemetry, flipper, shooterLeft, launchFlapLeft, shooterRight, launchFlapRight, this);
                    follower.followPath(MOVETOCOLLECT, Shooter.maxPower, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(COLLECT1, Shooter.maxPower, true);
                    timer.reset();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    flipper.setPosition(1);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT2, Shooter.maxPower, true);
                        timer.reset();
                        setPathState(4);
                    }

                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    flipper.setPosition(0);
                    if (timer.seconds() > 1) {
                        flipper.setPosition(0.525);
                        follower.followPath(COLLECT3, Shooter.maxPower, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {

                    follower.followPath(MOVETOLAUNCH2, Shooter.maxPower, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    Shooter.fireVolleySorted(limelight, telemetry, flipper, shooterLeft, launchFlapLeft, shooterRight, launchFlapRight, this);

                    follower.followPath(ENDOFFLINE, Shooter.maxPower, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
    }
}