package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.util.Globals.Poses.*;
import static org.firstinspires.ftc.teamcode.util.Globals.*;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.util.AutoOpMode;

@Autonomous(name = "Close 9 ball | pre, r1, r2", group = "close", preselectTeleOp = "Teleop")
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

                new FollowPathCommand(follower, scoreToEnd)
        ));
    }

    public void buildPaths() {
            startToScore = follower.pathBuilder()
                    .addPath(line(pose(CLOSE_START), pose(CLOSE_SCORE)))
                    .setLinearHeadingInterpolation(heading(CLOSE_START), heading(CLOSE_SCORE))
                    .build();

            scoreToIntakeRow1 = follower.pathBuilder()
                    .addPath(curve(pose(CLOSE_SCORE), pose(95, 82.5), pose(R1_START)))
                    .setLinearHeadingInterpolation(heading(CLOSE_SCORE), heading(INTAKE_ROW))
                    .addPath(line(pose(R1_START), pose(R1_END)))
                    .setConstantHeadingInterpolation(heading(INTAKE_ROW))
                    .build();

            row1ToScore = follower.pathBuilder()
                    .addPath(line(pose(R1_END), pose(CLOSE_SCORE)))
                    .setLinearHeadingInterpolation(heading(INTAKE_ROW), heading(CLOSE_SCORE))
                    .build();

            scoreToIntakeRow2 = follower.pathBuilder()
                    .addPath(curve(pose(CLOSE_SCORE), pose(95, 59), pose(R2_START)))
                    .setLinearHeadingInterpolation(heading(CLOSE_SCORE), heading(INTAKE_ROW))
                    .addPath(line(pose(R2_START), pose(R2_END)))
                    .setConstantHeadingInterpolation(heading(INTAKE_ROW))
                    .build();

            row2ToScore = follower.pathBuilder()
                    .addPath(line(pose(R2_END), pose(R2_START)))
                    .setConstantHeadingInterpolation(heading(INTAKE_ROW))
                    .addPath(line(pose(R2_END), pose(CLOSE_SCORE)))
                    .setLinearHeadingInterpolation(heading(INTAKE_ROW), heading(CLOSE_SCORE))
                    .build();

            scoreToEnd = follower.pathBuilder()
                    .addPath(line(pose(CLOSE_SCORE), pose(CLOSE_END)))
                    .setLinearHeadingInterpolation(heading(CLOSE_SCORE), heading(CLOSE_END))
                    .build();
    }
}
