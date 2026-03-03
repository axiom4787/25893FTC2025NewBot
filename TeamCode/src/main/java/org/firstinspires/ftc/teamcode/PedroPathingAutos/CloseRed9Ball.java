package org.firstinspires.ftc.teamcode.PedroPathingAutos;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Close Red | 9 ball", group = "close")
@Configurable // Panels
public class CloseRed9Ball extends OpMode {
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
        follower.setStartingPose(new Pose(120, 129, Math.toRadians(36)));

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
        public PathChain balls1ToShootPos;
        public PathChain shootPosToBalls1;
        public PathChain intakeBalls1;
        public PathChain balls2ToShootPos;
        public PathChain shootPosToBalls2;
        public PathChain intakeBalls2;

        public Paths(Follower follower) {
            startToShootPos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(120.000, 129.000),
                                    new Pose(96.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))
                    .build();

            shootPosToBalls1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 112.000),
                                    new Pose(102.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .build();

            intakeBalls1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(102.000, 84.000),
                                    new Pose(126.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            balls1ToShootPos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.000, 84.000),
                                    new Pose(96.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .build();

            shootPosToBalls2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 112.000),
                                    new Pose(102.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .setReversed()
                    .build();

            intakeBalls2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(102.000, 60.000),
                                    new Pose(133.000, 60.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            balls2ToShootPos = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.000, 60.000),
                                    new Pose(97.517, 56.369),
                                    new Pose(96.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .build();
        }
    }

    double actionStartTime = 0;

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.enableShooter();
                follower.followPath(paths.startToShootPos);
                pathState = 1;
                break;
            case 1:
                if (follower.isBusy()) break;

                robot.enableScoring();
                actionStartTime = time;
                pathState = 2;
                break;
            case 2:
                if (time - actionStartTime < robot.shootTime) break;

                robot.disableScoring();
                follower.followPath(paths.shootPosToBalls1);
                pathState = 3;
                break;
            case 3:
                if (follower.isBusy()) break;

                robot.enableIntake();
                follower.followPath(paths.intakeBalls1);
                pathState = 4;
                break;
            case 4:
                if (follower.isBusy()) break;

                robot.disableIntake();
                follower.followPath(paths.balls1ToShootPos);
                pathState = 5;
                break;
            case 5:
                if (follower.isBusy()) break;

                robot.enableScoring();
                actionStartTime = time;
                pathState = 6;
                break;
            case 6:
                if (time - actionStartTime < robot.shootTime) break;

                robot.disableScoring();
                follower.followPath(paths.shootPosToBalls2);
                pathState = 7;
                break;
            case 7:
                if (follower.isBusy()) break;

                robot.enableIntake();
                follower.followPath(paths.intakeBalls2);
                pathState = 8;
                break;
            case 8:
                if (follower.isBusy()) break;

                robot.disableIntake();
                follower.followPath(paths.balls2ToShootPos);
                pathState = 9;
                break;
            case 9:
                if (follower.isBusy()) break;

                robot.enableScoring();
                actionStartTime = time;
                pathState = 10;
                break;
            case 10:
                if (time - actionStartTime < robot.shootTime) break;

                robot.disableScoring();
                robot.disableShooter();
                pathState = -1;
                break;
        }

        return pathState;
    }
}