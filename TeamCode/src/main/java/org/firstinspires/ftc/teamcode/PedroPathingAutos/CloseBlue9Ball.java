package org.firstinspires.ftc.teamcode.PedroPathingAutos;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
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

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
        public PathChain goaltoshootpos;
        public PathChain shootpostoballs1;
        public PathChain intakeballs1;
        public PathChain balls1toshootpos;
        public PathChain shootpostoballs2;
        public PathChain intakeballs2;
        public PathChain balls2toshootpos;

        public Paths(Follower follower) {
            goaltoshootpos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.000, 129.000),
                                    new Pose(48.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                    .build();

            shootpostoballs1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(48.000, 112.000),
                                    new Pose(40.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            intakeballs1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(40.000, 84.000),
                                    new Pose(16.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            balls1toshootpos = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.000, 84.000),
                                    new Pose(46.840, 88.846),
                                    new Pose(48.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            shootpostoballs2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(48.000, 112.000),
                                    new Pose(40.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            intakeballs2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(40.000, 60.000),
                                    new Pose(9.000, 60.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            balls2toshootpos = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.000, 60.000),
                                    new Pose(46.483, 56.369),
                                    new Pose(48.000, 112.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // TODO: Start shooter
                follower.followPath(paths.goaltoshootpos);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    // TODO: Score balls
                    follower.followPath(paths.shootpostoballs1);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    // TODO: Start intaking
                    follower.followPath(paths.intakeballs1);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    // TODO: Stop intaking
                    follower.followPath(paths.balls1toshootpos);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    // TODO: Score balls
                    follower.followPath(paths.shootpostoballs2);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // TODO: Start intaking
                    follower.followPath(paths.intakeballs2);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    // TODO: Stop intaking
                    follower.followPath(paths.balls2toshootpos);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    // TODO: Score balls

                    pathState = -1;
                }
                break;
            case -1:
                // TODO: Stop shooter
                break;
        }
        return pathState;
    }
}