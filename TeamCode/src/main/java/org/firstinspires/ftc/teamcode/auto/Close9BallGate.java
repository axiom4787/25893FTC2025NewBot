package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.Globals.m;

import org.firstinspires.ftc.teamcode.util.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.Globals;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

@Autonomous(name = "Close 9 ball gate | pre, r2, gate", group = "close", preselectTeleOp = "Teleop")
@Configurable
public class Close9BallGate extends AutoOpMode {
    PathChain startToScore, scoreToEnd,
            scoreToIntakeRow2, row2ToScore,
            scoreToGate, gateToScore;

    public void scheduleAutoSequence() {
        schedule(new SequentialCommandGroup(
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, startToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::idle),
                new InstantCommand(intake::intake),

                new FollowPathCommand(follower, scoreToIntakeRow2),
                new InstantCommand(intake::off),
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, row2ToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::idle),
                new InstantCommand(intake::intake),

                new FollowPathCommand(follower, scoreToGate),
                new InstantCommand(intake::off),
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, gateToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::off),
                new InstantCommand(intake::off),

                new FollowPathCommand(follower, scoreToEnd),
                Globals.saveAutoEndPose(follower)
        ));
    }

    public void buildPaths() {
        startToScore = Globals.Close.START_TO_SCORE(follower);

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

        scoreToGate = follower.pathBuilder()
                .addPath(new BezierCurve(Globals.Close.SCORE_POSE, m(new Pose(104, 73)), Globals.Misc.GATE_INTAKE))
                .setLinearHeadingInterpolation(Globals.Close.SCORE_POSE.getHeading(), Globals.Misc.GATE_INTAKE.getHeading(), 0.8)
                .build();

        gateToScore = follower.pathBuilder()
                .addPath(new BezierCurve(Globals.Misc.GATE_INTAKE, m(new Pose(104, 73)), Globals.Close.SCORE_POSE))
                .setLinearHeadingInterpolation(Globals.Misc.GATE_INTAKE.getHeading(), Globals.Close.SCORE_POSE.getHeading())
                .build();

        scoreToEnd = Globals.Close.SCORE_TO_END(follower);
    }
}
