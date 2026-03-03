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

@Autonomous(name = "Far Blue | 6 ball", group = "far")
@Configurable // Panels
public class FarBlue6Ball extends OpMode {
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
        follower.setStartingPose(new Pose(60, 8, Math.toRadians(90)));

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

        public Paths(Follower follower) {
            startToShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 8.000), new Pose(60.000, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            shootPosToBalls3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 20.000), new Pose(40.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            intakeBalls3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(40.000, 36.000), new Pose(9.000, 36.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            balls3ToShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(9.000, 36.000), new Pose(60.000, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();
        }
    }

    public double actionStartTime = 0;

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                config.linearActuator.setPosition(0.25);
                config.shooter.setPower(1.0); // make it 1 so it hits all the shots
                config.turretServoLeft.setPower(0.0);
                follower.followPath(paths.startToShootPos);
                pathState = 1;
                break;
            case 1:
                if (follower.isBusy()) break;

                actionStartTime = time;
                pathState = 2;
                break;
            case 2:
                if (time - actionStartTime < 1.5) break;

                robot.enableScoring(0.75, 0.35);
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
                if (time - actionStartTime < 1.0) break;

                robot.enableScoring(0.75, 0.35);
                pathState = 8;
                break;
            case 8:
                if (time - actionStartTime < robot.farShootTime) break;

                robot.disableScoring();
                robot.disableShooter();
                pathState = -1;
                break;
            case -1:
                break;
        }

        return pathState;
    }
}