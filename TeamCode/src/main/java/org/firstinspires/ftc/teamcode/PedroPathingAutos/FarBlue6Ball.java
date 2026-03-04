package org.firstinspires.ftc.teamcode.PedroPathingAutos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

@Autonomous(name = "Far Blue | 6 ball", group = "far")
@Configurable // Panels
public class FarBlue6Ball extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class
    private final Config config = new Config();
    private RobotControls robot;

    @Override
    public void initialize() {
        super.reset();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        config.init(hardwareMap);
        robot = new RobotControls(config);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 8, Math.toRadians(90)));
        paths = new Paths(follower); // Build paths

        SequentialCommandGroup auto = new SequentialCommandGroup(
                // Enable shooter, setup hood
                new InstantCommand(robot::enableShooterFar),

                // Go to shooting position
                new FollowPathCommand(follower, paths.startToShootPos),

                // Wait, then shoot preloaded balls
                new WaitCommand(robot.farRevTime),
                new InstantCommand(robot::enableShooterFar),
                new WaitCommand(robot.farShootTime),
                new InstantCommand(robot::disableScoring),

                // Go to 3rd line of balls
                new FollowPathCommand(follower, paths.shootPosToBalls3),

                // Intake 3rd line of balls
                new InstantCommand(robot::enableIntake),
                new FollowPathCommand(follower, paths.intakeBalls3),
                new InstantCommand(robot::disableIntake),

                // Go to shooting position
                new FollowPathCommand(follower, paths.balls3ToShootPos),

                // Wait, then shoot balls
                new WaitCommand(robot.farRevTime2),
                new InstantCommand(robot::enableScoring),
                new WaitCommand(robot.farShootTime),
                new InstantCommand(robot::disableScoring),

                // Disable shooter
                new InstantCommand(robot::disableShooter),

                // Leave the launch zone
                new FollowPathCommand(follower, paths.leave)
        );

        schedule(auto);
    }

    @Override
    public void run() {
        super.run();

        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update();
    }

    public static class Paths {
        public PathChain startToShootPos;
        public PathChain shootPosToBalls3;
        public PathChain intakeBalls3;
        public PathChain balls3ToShootPos;
        public PathChain leave;

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

            leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 20.000), new Pose(36.000, 10.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(90))
                    .build();
        }
    }
}