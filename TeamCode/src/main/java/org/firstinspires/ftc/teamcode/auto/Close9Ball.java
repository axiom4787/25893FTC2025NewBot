package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.Globals.m;

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

import org.firstinspires.ftc.teamcode.util.CommandOpModeWithAlliance;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close 9 ball | pre, r1, r2", group = "close", preselectTeleOp = "Teleop")
@Configurable
public class Close9Ball extends CommandOpModeWithAlliance {
    public Follower follower;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    PathChain startToScore, scoreToEnd,
            scoreToIntakeRow1, row1ToScore,
            scoreToIntakeRow2, row2ToScore;

    Hood hood;
    Intake intake;
    Vision vision;
    Shooter shooter;
    Turret turret;

    @Override
    public void initialize() {
        super.reset();

        Hardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        hood = new Hood();
        intake = new Intake();
        vision = new Vision();
        shooter = new Shooter();
        turret = new Turret();
    }

    private boolean hasStarted = false;
    @Override
    public void run() {
        if (!hasStarted) {
            hasStarted = true;

            Globals.setAlliance(alliance);
            follower.setStartingPose(Globals.Close.START_POSE);
            buildPaths();
            scheduleAuto();
        }

        super.run();
        follower.update();

        turret.update(follower);
        hood.update(follower);
        shooter.update(follower);

        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void scheduleAuto() {
        schedule(new SequentialCommandGroup(
                new InstantCommand(shooter::setShoot),
                new FollowPathCommand(follower, startToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::setIdle),
                new InstantCommand(intake::intake),

                new FollowPathCommand(follower, scoreToIntakeRow1),
                new InstantCommand(intake::off),
                new InstantCommand(shooter::setShoot),
                new FollowPathCommand(follower, row1ToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::setIdle),
                new InstantCommand(intake::intake),

                new FollowPathCommand(follower, scoreToIntakeRow2),
                new InstantCommand(intake::off),
                new InstantCommand(shooter::setShoot),
                new FollowPathCommand(follower, row2ToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::setOff),
                new InstantCommand(intake::off),

                new FollowPathCommand(follower, scoreToEnd),
                Globals.saveAutoEndPose(follower)
        ));
    }

    public void buildPaths() {
            startToScore = Globals.Close.START_TO_SCORE(follower);

            scoreToIntakeRow1 = follower.pathBuilder()
                    .addPath(new BezierCurve(Globals.Close.SCORE_POSE, m(new Pose(95, 82.5)), Globals.Artifacts.ROW_1_START))
                    .setLinearHeadingInterpolation(Globals.Close.SCORE_POSE.getHeading(), Globals.Artifacts.INTAKE_HEADING)
                    .addPath(new BezierLine(Globals.Artifacts.ROW_1_START, Globals.Artifacts.ROW_1_END))
                    .setConstantHeadingInterpolation(Globals.Artifacts.INTAKE_HEADING)
                    .build();

            row1ToScore = follower.pathBuilder()
                    .addPath(new BezierLine(Globals.Artifacts.ROW_1_END, Globals.Close.SCORE_POSE))
                    .setLinearHeadingInterpolation(Globals.Artifacts.INTAKE_HEADING, Globals.Close.SCORE_POSE.getHeading())
                    .build();

            scoreToIntakeRow2 = follower.pathBuilder()
                    .addPath(new BezierCurve(Globals.Close.SCORE_POSE, m(new Pose(95, 59)), Globals.Artifacts.ROW_2_START))
                    .setLinearHeadingInterpolation(Globals.Close.SCORE_POSE.getHeading(), Globals.Artifacts.INTAKE_HEADING)
                    .addPath(new BezierLine(Globals.Artifacts.ROW_2_START, Globals.Artifacts.ROW_2_END))
                    .setConstantHeadingInterpolation(Globals.Artifacts.INTAKE_HEADING)
                    .build();

            row2ToScore = follower.pathBuilder()
                    .addPath(new BezierLine(Globals.Artifacts.ROW_2_END, Globals.Artifacts.ROW_2_START))
                    .setConstantHeadingInterpolation(Globals.Artifacts.INTAKE_HEADING)
                    .addPath(new BezierLine(Globals.Artifacts.ROW_2_START, Globals.Close.SCORE_POSE))
                    .setLinearHeadingInterpolation(Globals.Artifacts.INTAKE_HEADING, Globals.Close.SCORE_POSE.getHeading())
                    .build();

            scoreToEnd = Globals.Close.SCORE_TO_END(follower);
    }
}
