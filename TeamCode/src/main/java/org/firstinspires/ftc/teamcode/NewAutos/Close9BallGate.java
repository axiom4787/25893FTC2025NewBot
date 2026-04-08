package org.firstinspires.ftc.teamcode.NewAutos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Boilerplate.CommandOpModeWithAlliance;
import org.firstinspires.ftc.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Test Close 9 ball gate", group = "close")
@Configurable
public class Close9BallGate extends CommandOpModeWithAlliance {
    public Follower follower;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    PathChain startToScore, scoreToEnd,
            scoreToRow2, intakeRow2, row2ToScore,
            scoreToGate, gateToScore;

    HoodSubsystem hoodSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimeLightSubsystem limeLightSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    @Override
    public void initialize() {
        super.reset();

        Hardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        hoodSubsystem = new HoodSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        limeLightSubsystem = new LimeLightSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        turretSubsystem = new TurretSubsystem();
    }

    private boolean hasStarted = false;
    @Override
    public void run() {
        if (!hasStarted) {
            hasStarted = true;

            Shared2.setAlliance(alliance);
            follower.setStartingPose(Shared2.Close.START_POSE);
            buildPaths(follower);
            scheduleAuto();
        }

        super.run();
        follower.update();

        // TODO: Auto aim at the goal

        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void scheduleAuto() {
        schedule(
                new FollowPathCommand(follower, startToScore),
                // TODO: Score

                new FollowPathCommand(follower, scoreToRow2),
                new FollowPathCommand(follower, intakeRow2),
                new FollowPathCommand(follower, row2ToScore),
                // TODO: Score

                new FollowPathCommand(follower, scoreToGate),
                new FollowPathCommand(follower, gateToScore),
                // TODO: Score

                new FollowPathCommand(follower, scoreToEnd),
                Shared2.saveAutoEndPose(follower)
        );
    }

    public void buildPaths(Follower follower) {
        startToScore = Shared2.Close.START_TO_SCORE(follower);

        scoreToRow2 = follower.pathBuilder()
                .addPath(new BezierLine(Shared2.Close.SCORE_POSE, Shared2.Artifacts.ROW_2_START))
                .setLinearHeadingInterpolation(Shared2.Close.SCORE_POSE.getHeading(), Shared2.Artifacts.INTAKE_HEADING)
                .build();

        intakeRow2 = Shared2.Artifacts.INTAKE_ROW_2(follower);

        row2ToScore = follower.pathBuilder()
                .addPath(new BezierLine(Shared2.Artifacts.ROW_2_END, Shared2.Artifacts.ROW_2_START))
                .setConstantHeadingInterpolation(Shared2.Artifacts.INTAKE_HEADING)
                .addPath(new BezierLine(Shared2.Artifacts.ROW_2_START, Shared2.Close.SCORE_POSE))
                .setLinearHeadingInterpolation(Shared2.Artifacts.INTAKE_HEADING, Shared2.Close.SCORE_POSE.getHeading())
                .build();

        scoreToGate = follower.pathBuilder()
                .addPath(new BezierCurve(Shared2.Close.SCORE_POSE, new Pose(104, 73), Shared2.Misc.GATE_INTAKE))
                .setLinearHeadingInterpolation(Shared2.Close.SCORE_POSE.getHeading(), Shared2.Misc.GATE_INTAKE.getHeading())
                .build();

        gateToScore = follower.pathBuilder()
                .addPath(new BezierCurve(Shared2.Misc.GATE_INTAKE, new Pose(104, 73), Shared2.Close.SCORE_POSE))
                .setLinearHeadingInterpolation(Shared2.Misc.GATE_INTAKE.getHeading(), Shared2.Close.SCORE_POSE.getHeading())
                .build();

        scoreToEnd = Shared2.Close.SCORE_TO_END(follower);
    }
}
