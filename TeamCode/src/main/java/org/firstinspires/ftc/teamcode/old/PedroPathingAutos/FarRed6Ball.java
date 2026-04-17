package org.firstinspires.ftc.teamcode.old.PedroPathingAutos;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.old.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Far Red | 6 ball", group = "far")
@Configurable // Panels
@Disabled
public class FarRed6Ball extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private final Config config = new Config();
    private RobotControls robot;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        config.init(hardwareMap);
        robot = new RobotControls(config);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(84, 8.5, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain startToShootPos;
        public PathChain shootPosToBalls3;
        public PathChain intakeBalls3;
        public PathChain balls3ToShootPos;
        public PathChain leave;

        public Paths(Follower follower) {
            startToShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 8.500), new Pose(84.000, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))
                    .build();

            shootPosToBalls3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 20.000), new Pose(104.000, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                    .build();

            intakeBalls3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(104.000, 35.000), new Pose(135.000, 35.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            balls3ToShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(135.000, 35.000), new Pose(84.000, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(65))
                    .build();

            leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(84.000, 20.000), new Pose(108.000, 8.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))
                    .build();
        }
    }

    private double actionStartTime = 0;

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.enableShooterFar();
                follower.followPath(paths.startToShootPos);
                pathState = 1;
                break;
            case 1:
                if (follower.isBusy()) break;

                actionStartTime = time;
                pathState = 2;
                break;
            case 2:
                if (time - actionStartTime < robot.farRevTime) break;

                robot.enableScoringFar();
                pathState = 3;
                break;
            case 3:
                if (time - actionStartTime < robot.farShootTime) break;

                robot.disableScoring();
                follower.followPath(paths.shootPosToBalls3);
                pathState = 4;
                break;
            case 4:
                if (follower.isBusy()) break;

                robot.enableIntake();
                follower.followPath(paths.intakeBalls3);
                pathState = 5;
                break;
            case 5:
                if (follower.isBusy()) break;
                robot.disableIntake();

                follower.followPath(paths.balls3ToShootPos);
                pathState = 6;
                break;
            case 6:
                if (follower.isBusy()) break;

                actionStartTime = time;
                pathState = 7;
                break;
            case 7:
                if (time - actionStartTime < robot.farRevTime2) break;

                robot.enableScoringFar();
                pathState = 8;
                break;
            case 8:
                if (time - actionStartTime < robot.farShootTime) break;

                robot.disableScoring();
                robot.disableShooter();
                pathState = 9;
                break;
            case 9:
                follower.followPath(paths.leave);
                pathState = -1;
            case -1:
                break;
        }

        robot.updateSmartServo();
        return pathState;
    }
}