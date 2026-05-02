package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.Globals.Poses.*;
import static org.firstinspires.ftc.teamcode.util.Globals.*;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.util.AutoOpMode;

@Autonomous(name = "Far 9 ball | pre, r3, hp", group = "far", preselectTeleOp = "Teleop")
public class Far9Ball extends AutoOpMode {
    PathChain startToScore, scoreToEnd,
            scoreToIntakeRow3, row3ToScore,
            scoreToCorner, cornerToScore;

    HeadingInterpolator row3Interpolator = HeadingInterpolator.piecewise(
            new HeadingInterpolator.PiecewiseNode(0.0, 0.3, HeadingInterpolator.linear(heading(0), heading(23))),
            new HeadingInterpolator.PiecewiseNode(0.3, 0.7, HeadingInterpolator.constant(heading(23))),
            new HeadingInterpolator.PiecewiseNode(0.7, 1.0, HeadingInterpolator.linear(heading(23), heading(0)))
    );

    @Override
    public void buildPaths() {
        startToScore = follower.pathBuilder()
                .addPath(line(pose(FAR_START), pose(FAR_SCORE)))
                .setLinearHeadingInterpolation(heading(FAR_START), heading(90))
                .build();

        scoreToIntakeRow3 = follower.pathBuilder()
                .addPath(curve(pose(FAR_SCORE), pose(90, 35.5), pose(R3_START)))
                .setLinearHeadingInterpolation(heading(90), heading(INTAKE_ROW))
                .addPath(line(pose(R3_START), pose(R3_END)))
                .setConstantHeadingInterpolation(heading(INTAKE_ROW))
                .build();

        row3ToScore = follower.pathBuilder()
                .addPath(line(pose(R3_END), pose(FAR_SCORE)))
                .setHeadingInterpolation(row3Interpolator)
                .build();

        scoreToCorner = follower.pathBuilder()
                .addPath(line(pose(FAR_SCORE), pose(CORNER_SPIKE)))
                .setLinearHeadingInterpolation(heading(FAR_SCORE), heading(CORNER_SPIKE))
                .build();

        cornerToScore = follower.pathBuilder()
                .addPath(line(pose(CORNER_SPIKE), pose(FAR_SCORE)))
                .setLinearHeadingInterpolation(heading(CORNER_SPIKE), heading(FAR_SCORE))
                .build();

        scoreToEnd = follower.pathBuilder()
                .addPath(line(pose(FAR_SCORE), pose(FAR_END)))
                .setLinearHeadingInterpolation(heading(FAR_SCORE), heading(FAR_END))
                .build();
    }

    @Override
    public void scheduleAutoSequence() {
        schedule(new SequentialCommandGroup(
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, startToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::idle),
                new InstantCommand(intake::intake),

                new FollowPathCommand(follower, scoreToIntakeRow3),
                new InstantCommand(intake::off),
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, row3ToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
                new InstantCommand(shooter::idle),
                new InstantCommand(intake::intake),

                new FollowPathCommand(follower, scoreToCorner),
                new InstantCommand(intake::off),
                new InstantCommand(shooter::shoot),
                new FollowPathCommand(follower, cornerToScore),

                new InstantCommand(intake::index),
                new WaitCommand(3000),
//                new InstantCommand(shooter::idle),
//                new InstantCommand(intake::intake),
//
//                new FollowPathCommand(follower, scoreToCorner),
//                new InstantCommand(intake::off),
//                new InstantCommand(shooter::shoot),
//                new FollowPathCommand(follower, cornerToScore),
//
//                new InstantCommand(intake::index),
//                new WaitCommand(3000),
                new InstantCommand(shooter::off),
                new InstantCommand(intake::off)
        ));
    }
}
