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

@Autonomous(name = "Close 9 ball | pre, r1, r2", group = "close", preselectTeleOp = "Teleop")
@Configurable
public class Close9Ball extends AutoOpMode {
    PathChain startToScore, scoreToEnd,
            scoreToIntakeRow1, row1ToScore,
            scoreToIntakeRow2, row2ToScore;

    @Override
    public void scheduleAutoSequence() {
        schedule(new SequentialCommandGroup(
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, startToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::idle),
                new InstantCommand(intake::intake),

                new FollowPathCommand(follower, scoreToIntakeRow1),
                new InstantCommand(intake::off),
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, row1ToScore),

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
                new InstantCommand(shooter::off),
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
