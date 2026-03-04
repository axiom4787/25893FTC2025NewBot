package org.firstinspires.ftc.teamcode.PedroPathingAutos;
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
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

@Autonomous(name = "Close Blue | 9 ball", group = "close")
@Configurable // Panels
public class CloseBlue9Ball extends CommandOpMode {
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
        follower.setStartingPose(new Pose(24, 129, Math.toRadians(144)));
        paths = new Paths(follower); // Build paths

        SequentialCommandGroup auto = new SequentialCommandGroup(
                // Turn on shooter, setup hood
                new InstantCommand(robot::enableShooter),

                // Go from start to shoot position
                new FollowPathCommand(follower, paths.startToShootPos),

                // Score preload
                new InstantCommand(robot::enableScoring),
                new WaitCommand((long) robot.shootTime),
                new InstantCommand(robot::disableScoring),

                // Go to in front of line 1 of balls
                new FollowPathCommand(follower, paths.shootPosToBalls1),

                // Intake 1st line of balls
                new InstantCommand(robot::enableIntake),
                new FollowPathCommand(follower, paths.intakeBalls1),
                new InstantCommand(robot::disableIntake),

                // Go to shoot position
                new FollowPathCommand(follower, paths.balls1ToShootPos),

                // Score balls
                new InstantCommand(robot::enableScoring),
                new WaitCommand((long) robot.shootTime),
                new InstantCommand(robot::disableScoring),

                // Go to in front of line 2 of balls
                new FollowPathCommand(follower, paths.shootPosToBalls2),

                // Intake 2nd line of balls
                new InstantCommand(robot::enableIntake),
                new FollowPathCommand(follower, paths.intakeBalls2),
                new InstantCommand(robot::disableIntake),

                // Go to shoot position
                new FollowPathCommand(follower, paths.balls2ToShootPos),

                // Score balls
                new InstantCommand(robot::enableScoring),
                new WaitCommand((long) robot.shootTime),
                new InstantCommand(robot::disableScoring),

                // Disable shooter
                new InstantCommand(robot::disableShooter)
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
}