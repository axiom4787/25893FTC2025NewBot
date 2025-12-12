package org.firstinspires.ftc.teamcode.OpModes.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Autonomous - Ball Collection", group = "Autonomous")
public class AutonPedroDocumentation extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    // Starting position of the robot
    private final Pose startPose = new Pose(60.642, 8.721, Math.toRadians(180));

    // Path declarations
    private PathChain GettingNearestBalls;
    private PathChain GoingBacktoShoot;
    private PathChain GettingNextSetofBalls;
    private PathChain GoingBacktoShoot2;
    private PathChain GettingThirdSetofBalls;
    private PathChain Path6;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        // You can add initialization feedback here
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(GettingNearestBalls);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(GoingBacktoShoot);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(GettingNextSetofBalls);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(GoingBacktoShoot2);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(GettingThirdSetofBalls);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(Path6);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    // Set the state to a Case we won't use or define, so it just stops running any new paths
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        // Path 1: Get nearest balls (curved path)
        GettingNearestBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.642, 8.721),
                                new Pose(49.893, 37.318),
                                new Pose(23.324, 35.899)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 2: Return to shoot (straight line)
        GoingBacktoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(23.324, 35.899), new Pose(62.265, 24.135))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 3: Get next set of balls (curved path)
        GettingNextSetofBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.265, 24.135),
                                new Pose(53.341, 60.237),
                                new Pose(23.324, 60.034)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 4: Return to shoot (second time - straight line)
        GoingBacktoShoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(23.324, 60.034), new Pose(62.468, 23.932))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 5: Get third set of balls (curved path)
        GettingThirdSetofBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.468, 23.932),
                                new Pose(64.699, 84.169),
                                new Pose(23.932, 83.966)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 6: Final position (straight line)
        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(23.932, 83.966), new Pose(36.710, 94.715))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public void stop() {}
}