package org.firstinspires.ftc.teamcode.NewAutos;

import static org.firstinspires.ftc.teamcode.Boilerplate.Shared.m;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Boilerplate.CommandOpModeWithAlliance;
import org.firstinspires.ftc.teamcode.Boilerplate.Shared;
import org.firstinspires.ftc.teamcode.Hardware.CachingHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Test Close 9 ball", group = "close")
@Configurable
public class Close9Ball extends CommandOpModeWithAlliance {
    public Follower follower;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    PathChain startToScore, scoreToEnd,
            scoreToIntakeRow1, row1ToScore,
            scoreToIntakeRow2, row2ToScore;

    HoodSubsystem hoodSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimeLightSubsystem limeLightSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    @Override
    public void initialize() {
        super.reset();

        CachingHardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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
            buildPaths();
            scheduleAuto();
        }

        super.run();
        follower.update();

        turretSubsystem.update(follower);
        hoodSubsystem.update(follower);
        shooterSubsystem.update(follower);

        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void scheduleAuto() {
        schedule(new SequentialCommandGroup(
                new InstantCommand(shooterSubsystem::setShoot),
                new FollowPathCommand(follower, startToScore),

                new InstantCommand(intakeSubsystem::index),
                new WaitCommand(3000),
                new InstantCommand(shooterSubsystem::setIdle),
                new InstantCommand(intakeSubsystem::intake),

                new FollowPathCommand(follower, scoreToIntakeRow1),
                new InstantCommand(intakeSubsystem::off),
                new InstantCommand(shooterSubsystem::setShoot),
                new FollowPathCommand(follower, row1ToScore),

                new InstantCommand(intakeSubsystem::index),
                new WaitCommand(3000),
                new InstantCommand(shooterSubsystem::setIdle),
                new InstantCommand(intakeSubsystem::intake),

                new FollowPathCommand(follower, scoreToIntakeRow2),
                new InstantCommand(intakeSubsystem::off),
                new InstantCommand(shooterSubsystem::setShoot),
                new FollowPathCommand(follower, row2ToScore),

                new InstantCommand(intakeSubsystem::index),
                new WaitCommand(3000),
                new InstantCommand(shooterSubsystem::setOff),
                new InstantCommand(intakeSubsystem::off),

                new FollowPathCommand(follower, scoreToEnd),
                Shared.saveAutoEndPose(follower)
        ));
    }

    public void buildPaths() {
            startToScore = Shared.Close.START_TO_SCORE(follower);

            scoreToIntakeRow1 = follower.pathBuilder()
                    .addPath(new BezierCurve(Shared.Close.SCORE_POSE, m(new Pose(95, 82.5)), Shared.Artifacts.ROW_1_START))
                    .setLinearHeadingInterpolation(Shared.Close.SCORE_POSE.getHeading(), Shared.Artifacts.INTAKE_HEADING)
                    .addPath(new BezierLine(Shared.Artifacts.ROW_1_START, Shared.Artifacts.ROW_1_END))
                    .setConstantHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING)
                    .build();

            row1ToScore = follower.pathBuilder()
                    .addPath(new BezierLine(Shared.Artifacts.ROW_1_END, Shared.Close.SCORE_POSE))
                    .setLinearHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING, Shared.Close.SCORE_POSE.getHeading())
                    .build();

            scoreToIntakeRow2 = follower.pathBuilder()
                    .addPath(new BezierCurve(Shared.Close.SCORE_POSE, m(new Pose(95, 59)), Shared.Artifacts.ROW_2_START))
                    .setLinearHeadingInterpolation(Shared.Close.SCORE_POSE.getHeading(), Shared.Artifacts.INTAKE_HEADING)
                    .addPath(new BezierLine(Shared.Artifacts.ROW_2_START, Shared.Artifacts.ROW_2_END))
                    .setConstantHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING)
                    .build();

            row2ToScore = follower.pathBuilder()
                    .addPath(new BezierLine(Shared.Artifacts.ROW_2_END, Shared.Artifacts.ROW_2_START))
                    .setConstantHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING)
                    .addPath(new BezierLine(Shared.Artifacts.ROW_2_START, Shared.Close.SCORE_POSE))
                    .setLinearHeadingInterpolation(Shared.Artifacts.INTAKE_HEADING, Shared.Close.SCORE_POSE.getHeading())
                    .build();

            scoreToEnd = Shared.Close.SCORE_TO_END(follower);
    }
}
