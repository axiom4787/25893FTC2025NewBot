package org.firstinspires.ftc.teamcode.NewAutos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Boilerplate.CommandOpModeWithAlliance;
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
                new InstantCommand(shooterSubsystem::setShoot),
                new FollowPathCommand(follower, startToScore),

                new InstantCommand(intakeSubsystem::index),
                new WaitCommand(3000),
                new InstantCommand(intakeSubsystem::off),
                new InstantCommand(shooterSubsystem::setIdle),

                new FollowPathCommand(follower, scoreToRow1),
                new FollowPathCommand(follower, intakeRow1),
                new InstantCommand(shooterSubsystem::setShoot),
                new FollowPathCommand(follower, row1ToScore),

                new InstantCommand(intakeSubsystem::index),
                new WaitCommand(3000),
                new InstantCommand(intakeSubsystem::off),
                new InstantCommand(shooterSubsystem::setIdle),

                new FollowPathCommand(follower, scoreToRow2),
                new FollowPathCommand(follower, intakeRow2),
                new InstantCommand(shooterSubsystem::setShoot),
                new FollowPathCommand(follower, row1ToScore),

                new InstantCommand(intakeSubsystem::index),
                new WaitCommand(3000),
                new InstantCommand(intakeSubsystem::off),
                new InstantCommand(shooterSubsystem::setIdle),

                new FollowPathCommand(follower, scoreToRow3),
                new FollowPathCommand(follower, intakeRow3),
                new InstantCommand(shooterSubsystem::setShoot),
                new FollowPathCommand(follower, row3ToScore),

                new InstantCommand(intakeSubsystem::index),
                new WaitCommand(3000),
                new InstantCommand(intakeSubsystem::off),
                new InstantCommand(shooterSubsystem::setOff),

                new FollowPathCommand(follower, scoreToEnd),
                Shared2.saveAutoEndPose(follower)
        );
    }

    public void buildPaths(Follower follower) {
        startToScore = Shared2.Close.START_TO_SCORE(follower);

        scoreToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(Shared2.Close.SCORE_POSE, Shared2.Artifacts.ROW_1_START))
                .setLinearHeadingInterpolation(Shared2.Close.SCORE_POSE.getHeading(), Shared2.Artifacts.INTAKE_HEADING)
                .build();

        intakeRow1 = Shared2.Artifacts.INTAKE_ROW_1(follower);

        row1ToScore = follower.pathBuilder()
                .addPath(new BezierLine(Shared2.Artifacts.ROW_1_END, Shared2.Close.SCORE_POSE))
                .setLinearHeadingInterpolation(Shared2.Artifacts.INTAKE_HEADING, Shared2.Close.SCORE_POSE.getHeading())
                .build();

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

        scoreToRow3 = follower.pathBuilder()
                .addPath(new BezierLine(Shared2.Close.SCORE_POSE, Shared2.Artifacts.ROW_3_START))
                .setLinearHeadingInterpolation(Shared2.Close.SCORE_POSE.getHeading(), Shared2.Artifacts.INTAKE_HEADING)
                .build();

        intakeRow3 = Shared2.Artifacts.INTAKE_ROW_3(follower);

        row3ToScore = follower.pathBuilder()
                .addPath(new BezierLine(Shared2.Artifacts.ROW_3_END, Shared2.Far.SCORE_POSE))
                .setLinearHeadingInterpolation(Shared2.Artifacts.INTAKE_HEADING, Shared2.Far.SCORE_POSE.getHeading())
                .build();

        scoreToEnd = Shared2.Close.SCORE_TO_END(follower);
    }
}
