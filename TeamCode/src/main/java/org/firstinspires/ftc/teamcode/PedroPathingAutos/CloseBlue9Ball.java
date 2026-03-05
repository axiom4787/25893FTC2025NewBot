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

@Autonomous(name = "Close Blue | 9 ball", group = "close")
@Configurable // Panels
public class CloseBlue9Ball extends OpMode {
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
        follower.setStartingPose(new Pose(24, 129, Math.toRadians(144)));

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
        public PathChain shootPosToBalls1;
        public PathChain intakeBalls1;
        public PathChain balls1ToShootPos;
        public PathChain shootPosToBalls2;
        public PathChain intakeBalls2;
        public PathChain balls2ToShootPos;

        public Paths(Follower follower) {
            startToShootPos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.000, 129.000),
                                    new Pose(48.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                    .build();

            shootPosToBalls1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(48.000, 112.000),
                                    new Pose(42.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            intakeBalls1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 84.000),
                                    new Pose(18.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            balls1ToShootPos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.000, 84.000),
                                    new Pose(48.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            shootPosToBalls2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(48.000, 112.000),
                                    new Pose(42.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            intakeBalls2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000),
                                    new Pose(11.000, 60.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            balls2ToShootPos = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.000, 60.000),
                                    new Pose(46.483, 56.369),
                                    new Pose(48.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();
        }
    }

    double actionStartTime = 0;

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // enable shooter, go to first shooting position
                robot.enableShooter();
                follower.followPath(paths.startToShootPos);
                pathState = 1;
                break;
            case 1: // wait until at shooting position, then feed into shooter
                if (follower.isBusy()) break;

                robot.enableScoring();
                actionStartTime = time;
                pathState = 2;
                break;
            case 2: // wait until 3.5 seconds elapsed, then disable feeder and go to first row of balls
                if (time - actionStartTime < robot.shootTime) break;

                robot.disableScoring();
                follower.followPath(paths.shootPosToBalls1);
                pathState = 3;
                break;
            case 3: // wait until at the row of balls, then enable the intake and move along the balls
                if (follower.isBusy()) break;

                robot.enableIntake();
                follower.followPath(paths.intakeBalls1);
                pathState = 4;
                break;
            case 4: // wait until moved along balls, then disable the intake and move back toward shooting pos
                if (follower.isBusy()) break;

                robot.disableIntake();
                follower.followPath(paths.balls1ToShootPos);
                pathState = 5;
                break;
            case 5: // wait until at position, then start feeding
                if (follower.isBusy()) break;

                robot.enableScoring();
                actionStartTime = time;
                pathState = 6;
                break;
            case 6: // wait until 3.5 seconds elapsed, then disable feeder and go to second row of balls
                if (time - actionStartTime < robot.shootTime) break;

                robot.disableScoring();
                follower.followPath(paths.shootPosToBalls2);
                pathState = 7;
                break;
            case 7: // wait until at second row, then start intaking and move down the row
                if (follower.isBusy()) break;

                robot.enableIntake();
                follower.followPath(paths.intakeBalls2);
                pathState = 8;
                break;
            case 8: // wait until down row, then disable intake and move back to shoot pos
                if (follower.isBusy()) break;

                robot.disableIntake();
                follower.followPath(paths.balls2ToShootPos);
                pathState = 9;
                break;
            case 9: // wait until at shoot pos, then enable feeding
                if (follower.isBusy()) break;

                robot.enableScoring();
                actionStartTime = time;
                pathState = 10;
                break;
            case 10: // wait until 3.5 seconds elapsed, then disable feed and shooter. path is done
                if (time - actionStartTime < robot.shootTime) break;

                robot.disableScoring();
                robot.disableShooter();
                pathState = -1;
                break;
        }

        return pathState;
    }
}