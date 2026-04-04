package org.firstinspires.ftc.teamcode.NewAutos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Solo 12 ball", group = "solo")
@Configurable
public class Solo12Ball extends CommandOpModeWithAlliance {
    public Follower follower;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    PathChain startToScore, scoreToEnd,
            scoreToRow1, intakeRow1, row1ToScore,
            scoreToRow2, intakeRow2, row2ToScore,
            scoreToRow3, intakeRow3, row3ToScore;

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

            Shared.setAlliance(alliance);
            follower.setStartingPose(Shared.Close.START_POSE);
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

                new FollowPathCommand(follower, scoreToRow1),
                new FollowPathCommand(follower, intakeRow1),
                new FollowPathCommand(follower, row1ToScore),
                // TODO: Score

                new FollowPathCommand(follower, scoreToRow2),
                new FollowPathCommand(follower, intakeRow2),
                new FollowPathCommand(follower, row1ToScore),
                // TODO: Score

                new FollowPathCommand(follower, scoreToRow3),
                new FollowPathCommand(follower, intakeRow3),
                new FollowPathCommand(follower, row3ToScore),
                // TODO: Score

                new FollowPathCommand(follower, scoreToEnd),
                Shared.saveAutoEndPose(follower)
        );
    }

    public void buildPaths(Follower follower) {
        startToScore = Shared.Close.START_TO_SCORE(follower);

        scoreToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(Shared.Close.SCORE_POSE, Shared.Artifacts.ROW_1_START))
                .setLinearHeadingInterpolation(Shared.Close.SCORE_POSE.getHeading(), Shared.Artifacts.INTAKE_HEADING)
                .build();

        intakeRow1 = Shared.Artifacts.INTAKE_ROW_1(follower);

        row1ToScore = follower.pathBuilder()
                .addPath(new BezierLine(Shared.Artifacts.ROW_1_END, Shared.Close.SCORE_POSE))
                .setLinearHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING, Shared.Close.SCORE_POSE.getHeading())
                .build();

        scoreToRow2 = follower.pathBuilder()
                .addPath(new BezierLine(Shared.Close.SCORE_POSE, Shared.Artifacts.ROW_2_START))
                .setLinearHeadingInterpolation(Shared.Close.SCORE_POSE.getHeading(), Shared.Artifacts.INTAKE_HEADING)
                .build();

        intakeRow2 = Shared.Artifacts.INTAKE_ROW_2(follower);

        row2ToScore = follower.pathBuilder()
                .addPath(new BezierLine(Shared.Artifacts.ROW_2_END, Shared.Artifacts.ROW_2_START))
                .setConstantHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING)
                .addPath(new BezierLine(Shared.Artifacts.ROW_2_START, Shared.Close.SCORE_POSE))
                .setLinearHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING, Shared.Close.SCORE_POSE.getHeading())
                .build();

        scoreToRow3 = follower.pathBuilder()
                .addPath(new BezierLine(Shared.Close.SCORE_POSE, Shared.Artifacts.ROW_3_START))
                .setLinearHeadingInterpolation(Shared.Close.SCORE_POSE.getHeading(), Shared.Artifacts.INTAKE_HEADING)
                .build();

        intakeRow3 = Shared.Artifacts.INTAKE_ROW_3(follower);

        row3ToScore = follower.pathBuilder()
                .addPath(new BezierLine(Shared.Artifacts.ROW_3_END, Shared.Far.SCORE_POSE))
                .setLinearHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING, Shared.Far.SCORE_POSE.getHeading())
                .build();

        scoreToEnd = Shared.Close.SCORE_TO_END(follower);
    }
}
