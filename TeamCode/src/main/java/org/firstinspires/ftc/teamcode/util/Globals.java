package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.jetbrains.annotations.NotNull;

// use me not Shared ty
public class Globals {
    private Globals() {}

    public static final double FIELD_WIDTH = 141.5;

    public static Pose autoEndPose = new Pose(FIELD_WIDTH/2, FIELD_WIDTH/2);
    public static InstantCommand saveAutoEndPose(Follower follower) {
        return new InstantCommand(() -> autoEndPose = follower.getPose());
    }

    private static Alliance alliance = Alliance.RED;

    public static Alliance getAlliance() {
        return alliance;
    }

    public static void setAlliance(@NotNull Alliance alliance) {
        if (Globals.alliance != alliance) mirror();

        Globals.alliance = alliance;
    }

    public static Pose m(Pose pose) {
        return alliance == Alliance.RED ? pose : pose.mirror();
    }

    public static double m(double h) {
        return alliance == Alliance.RED ? h : Math.PI - h;
    }

    private static void mirror() {
        Close.START_POSE = Close.START_POSE.mirror();
        Close.SCORE_POSE = Close.SCORE_POSE.mirror();
        Close.END_POSE   = Close.END_POSE.mirror();

        Far.START_POSE   = Far.START_POSE.mirror();
        Far.SCORE_POSE   = Far.SCORE_POSE.mirror();
        Far.END_POSE     = Far.END_POSE.mirror();

        Artifacts.X_1_END = FIELD_WIDTH - Artifacts.X_1_END;
        Artifacts.X_2_END = FIELD_WIDTH - Artifacts.X_2_END;
        Artifacts.X_3_END = FIELD_WIDTH - Artifacts.X_3_END;
        Artifacts.X_START = FIELD_WIDTH - Artifacts.X_START;

        Artifacts.ROW_1_START = Artifacts.ROW_1_START.mirror();
        Artifacts.ROW_1_END   = Artifacts.ROW_1_END.mirror();
        Artifacts.ROW_2_START = Artifacts.ROW_2_START.mirror();
        Artifacts.ROW_2_END   = Artifacts.ROW_2_END.mirror();
        Artifacts.ROW_3_START = Artifacts.ROW_3_START.mirror();
        Artifacts.ROW_3_END   = Artifacts.ROW_3_END.mirror();
        Artifacts.INTAKE_HEADING = Math.PI - Artifacts.INTAKE_HEADING;

        Misc.FIELD_RELATIVE_DRIVE_HEADING_OFFSET = Math.PI - Misc.FIELD_RELATIVE_DRIVE_HEADING_OFFSET;
        Misc.GOAL_X = FIELD_WIDTH - Misc.GOAL_X;
        Misc.GOAL = Misc.GOAL.mirror();
        Misc.EDGE_INTAKE_Y = FIELD_WIDTH - Misc.EDGE_INTAKE_Y;
        Misc.CORNER_INTAKE = Misc.CORNER_INTAKE.mirror();
        Misc.OPEN_GATE     = Misc.OPEN_GATE.mirror();
        Misc.GATE_INTAKE   = Misc.GATE_INTAKE.mirror();
        Misc.GATE_INTAKE_APPROACH = Misc.GATE_INTAKE_APPROACH.mirror();
        Misc.PARK = Misc.PARK.mirror();
        Misc.CORNER_RELOCALIZE = Misc.CORNER_RELOCALIZE.mirror();
    }

    public static class Close {
        public static Pose START_POSE = new Pose(110, 132, Math.toRadians(0));
//        public static Pose SCORE_POSE = new Pose(106.25, 100, Math.toRadians(53));
        public static Pose SCORE_POSE = new Pose(95, 90, 0);
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
        public static double Y_1 = 82.5 - 1;
        public static double Y_2 = 59 - 3.5;
        public static double Y_3 = 35.5 - 5;

        public static double X_START = 101;

        public static double X_1_END = 125 + 2;
        public static double X_2_END = 131 - 0;
        public static double X_3_END = 131 - 0;

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
        public static double FIELD_RELATIVE_DRIVE_HEADING_OFFSET = Math.PI;

        public static double GOAL_X = 135;
        public static double GOAL_Y = 135;
        public static Pose GOAL = new Pose(GOAL_X, GOAL_Y);

        public static double EDGE_INTAKE_Y = 9;
        public static Pose CORNER_INTAKE = new Pose(132, EDGE_INTAKE_Y, Math.toRadians(0));

        public static Pose OPEN_GATE = new Pose(126, 69, Math.toRadians(0));
        public static Pose GATE_INTAKE = new Pose(129.5, 59.5, Math.toRadians(35));
        public static Pose GATE_INTAKE_APPROACH = new Pose(120, 59.5);

        public static Pose PARK = new Pose(38, 32.75, Math.toRadians(90));

        public static Pose CORNER_RELOCALIZE = new Pose(133.5, 8, Math.toRadians(90));
    }

    public static class Zones {
        private static final PolygonZone robotZone = new PolygonZone(16, 16);

        public static PolygonZone CLOSE_LAUNCH = new PolygonZone(
                new Point(0,     141.5),
                new Point(141.5, 141.5),
                new Point(70.75, 70.75)
        );

        public static PolygonZone FAR_LAUNCH = new PolygonZone(
                new Point(41.17, 0),
                new Point(94.33, 0),
                new Point(70.75, 23.58)
        );

        public static boolean isInLaunchZone() {
            return robotZone.isInside(CLOSE_LAUNCH) || robotZone.isInside(FAR_LAUNCH);
        }

        public static boolean isFullyInLaunchZone() {
            return robotZone.isFullyInside(CLOSE_LAUNCH) || robotZone.isFullyInside(FAR_LAUNCH);
        }

        public static boolean isNearLaunchZone() {
            double tol = 8;
            return distToLaunchZone() < tol;
        }

        public static void updateRobotLocation(Follower follower) {
            robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
            robotZone.setRotation(follower.getHeading());
        }

        public static double distToLaunchZone() {
            return Math.min(robotZone.distanceTo(CLOSE_LAUNCH), robotZone.distanceTo(FAR_LAUNCH));
        }
    }
}
