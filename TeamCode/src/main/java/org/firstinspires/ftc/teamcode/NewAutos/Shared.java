package org.firstinspires.ftc.teamcode.NewAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Boilerplate.Alliance;
import org.jetbrains.annotations.NotNull;

// so not use me! i assume the field is 144x144, which it is not. use Shared2 instead!!
public class Shared {
    private Shared() {}

    public static Pose autoEndPose = new Pose(72, 72);
    public static InstantCommand saveAutoEndPose(Follower follower) {
        return new InstantCommand(() -> autoEndPose = follower.getPose());
    }

    private static Alliance alliance = Alliance.RED;

    public static void setAlliance(@NotNull Alliance alliance) {
        if (Shared.alliance != alliance) mirror();

        Shared.alliance = alliance;
    }

    private static void mirror() {
        final double FIELD_WIDTH = 144;

        Close.START_POSE = Close.START_POSE.mirror(FIELD_WIDTH);
        Close.SCORE_POSE = Close.SCORE_POSE.mirror(FIELD_WIDTH);
        Close.END_POSE   = Close.END_POSE.mirror(FIELD_WIDTH);

        Far.START_POSE   = Far.START_POSE.mirror(FIELD_WIDTH);
        Far.SCORE_POSE   = Far.SCORE_POSE.mirror(FIELD_WIDTH);
        Far.END_POSE     = Far.END_POSE.mirror(FIELD_WIDTH);

        Artifacts.X_1_END = FIELD_WIDTH - Artifacts.X_1_END;
        Artifacts.X_2_END = FIELD_WIDTH - Artifacts.X_2_END;
        Artifacts.X_3_END = FIELD_WIDTH - Artifacts.X_3_END;
        Artifacts.X_START = FIELD_WIDTH - Artifacts.X_START;

        Artifacts.ROW_1_START = Artifacts.ROW_1_START.mirror(FIELD_WIDTH);
        Artifacts.ROW_1_END   = Artifacts.ROW_1_END.mirror(FIELD_WIDTH);
        Artifacts.ROW_2_START = Artifacts.ROW_2_START.mirror(FIELD_WIDTH);
        Artifacts.ROW_2_END   = Artifacts.ROW_2_END.mirror(FIELD_WIDTH);
        Artifacts.ROW_3_START = Artifacts.ROW_3_START.mirror(FIELD_WIDTH);
        Artifacts.ROW_3_END   = Artifacts.ROW_3_END.mirror(FIELD_WIDTH);
        Artifacts.INTAKE_HEADING = Math.PI - Artifacts.INTAKE_HEADING;

        Misc.GOAL_X = FIELD_WIDTH - Misc.GOAL_X;
        Misc.GOAL = Misc.GOAL.mirror(FIELD_WIDTH);
        Misc.EDGE_INTAKE_Y = FIELD_WIDTH - Misc.EDGE_INTAKE_Y;
        Misc.CORNER_INTAKE = Misc.CORNER_INTAKE.mirror(FIELD_WIDTH);
        Misc.OPEN_GATE     = Misc.OPEN_GATE.mirror(FIELD_WIDTH);
        Misc.GATE_INTAKE   = Misc.GATE_INTAKE.mirror(FIELD_WIDTH);
        Misc.GATE_INTAKE_APPROACH = Misc.GATE_INTAKE_APPROACH.mirror(FIELD_WIDTH);
        Misc.PARK = Misc.PARK.mirror(FIELD_WIDTH);
    }

    public static class Close {
        public static Pose START_POSE = new Pose(108, 136, Math.toRadians(0));
        public static Pose SCORE_POSE = new Pose(108, 100, Math.toRadians(53));
        public static Pose END_POSE   = new Pose(120, 100);

        public static PathChain START_TO_SCORE(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, SCORE_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SCORE_POSE.getHeading())
                    .build();
        }

        public static PathChain SCORE_TO_END(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(SCORE_POSE, END_POSE))
                    .setConstantHeadingInterpolation(SCORE_POSE.getHeading())
                    .build();
        }
    }

    public static class Far {
        public static Pose START_POSE = new Pose(84, 8.5, Math.toRadians(90));
        public static Pose SCORE_POSE = new Pose(84, 16, Math.toRadians(65));
        public static Pose END_POSE   = new Pose(108, 8.5, Math.toRadians(90));

        public static PathChain START_TO_SCORE(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, SCORE_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SCORE_POSE.getHeading())
                    .build();
        }

        public static PathChain SCORE_TO_END(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(SCORE_POSE, END_POSE))
                    .setLinearHeadingInterpolation(SCORE_POSE.getHeading(), END_POSE.getHeading())
                    .build();
        }
    }

    public static class Artifacts {
        public static double Y_1 = 84;
        public static double Y_2 = 60;
        public static double Y_3 = 36;

        public static double X_START = 102;

        public static double X_1_END = 128;
        public static double X_2_END = 134;
        public static double X_3_END = 134;

        public static double INTAKE_HEADING = Math.toRadians(0);

        public static Pose ROW_1_START = new Pose(X_START, Y_1, INTAKE_HEADING);
        public static Pose ROW_1_END   = new Pose(X_1_END, Y_1, INTAKE_HEADING);
        public static Pose ROW_2_START = new Pose(X_START, Y_2, INTAKE_HEADING);
        public static Pose ROW_2_END   = new Pose(X_2_END, Y_2, INTAKE_HEADING);
        public static Pose ROW_3_START = new Pose(X_START, Y_3, INTAKE_HEADING);
        public static Pose ROW_3_END   = new Pose(X_3_END, Y_3, INTAKE_HEADING);

        public static PathChain INTAKE_ROW_1(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(ROW_1_START, ROW_1_END))
                    .setConstantHeadingInterpolation(INTAKE_HEADING)
                    .build();
        }

        public static PathChain INTAKE_ROW_2(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(ROW_2_START, ROW_2_END))
                    .setConstantHeadingInterpolation(INTAKE_HEADING)
                    .build();
        }

        public static PathChain INTAKE_ROW_3(Follower follower) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(ROW_3_START, ROW_3_END))
                    .setConstantHeadingInterpolation(INTAKE_HEADING)
                    .build();
        }
    }

    public static class Misc {
        public static double GOAL_X = 136;
        public static double GOAL_Y = 136;
        public static Pose GOAL = new Pose(GOAL_X, GOAL_Y);

        public static double EDGE_INTAKE_Y = 9;
        public static Pose CORNER_INTAKE = new Pose(135, EDGE_INTAKE_Y, Math.toRadians(0));

        public static Pose OPEN_GATE = new Pose(128, 70, Math.toRadians(0));
        public static Pose GATE_INTAKE_APPROACH = new Pose(120, 60);
        public static Pose GATE_INTAKE = new Pose(133, 60, Math.toRadians(35));

        public static Pose PARK = new Pose(38, 32.75, Math.toRadians(90));
    }
}
