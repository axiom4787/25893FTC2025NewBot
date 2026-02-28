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

@Autonomous(name = "Far Blue | 6 ball", group = "far")
@Configurable // Panels
public class FarBlue6Ball extends OpMode {
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
        public PathChain starttoshootpos;
        public PathChain shootpostoballs3;
        public PathChain intakeballs3;
        public PathChain balls3toshootpos;

        public Paths(Follower follower) {
            starttoshootpos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(60.000, 8.000),
                                    new Pose(60.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113))
                    .build();

            shootpostoballs3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 20.000),
                                    new Pose(59.334, 35.666),
                                    new Pose(40.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113), Math.toRadians(180))
                    .build();

            intakeballs3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(40.000, 36.000),
                                    new Pose(9.000, 36.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            balls3toshootpos = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(9.000, 36.000),
                                    new Pose(60.000, 20.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // TODO: Start shooter
                follower.followPath(paths.starttoshootpos);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    // TODO: Shoot balls

                    follower.followPath(paths.shootpostoballs3);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    // TODO: Start intake

                    follower.followPath(paths.intakeballs3);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    // TODO: Stop intake

                    follower.followPath(paths.balls3toshootpos);
                    pathState = 4;
                }
                break;
            case 4:
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