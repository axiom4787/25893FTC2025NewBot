package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.jetbrains.annotations.NotNull;

public class Globals {
    private Globals() {}

    public static final double FIELD_WIDTH = 141.5;

    private static Pose savedPose = new Pose(FIELD_WIDTH / 2, FIELD_WIDTH / 2, Math.toRadians(0));

    public static Pose getSavedPose() {
        return savedPose;
    }

    private static Alliance alliance = Alliance.RED;

    public static void setAlliance(@NotNull Alliance alliance) {
        Globals.alliance = alliance;
    }

    // Poses, mirroring, lines, curves

    public static Pose pose(Poses pose) {
        return pose(pose.x, pose.y);
    }

    public static Pose pose(double x, double y) {
        return alliance == Alliance.RED ? new Pose(x, y) : new Pose(FIELD_WIDTH - x, y);
    }

    public static double heading(Poses pose) {
        return heading(pose.heading);
    }

    public static double heading(double heading) {
        return alliance == Alliance.RED ? Math.toRadians(heading) : Math.toRadians(180 - heading);
    }

    public static BezierLine line(Pose start, Pose end) {
        return new BezierLine(start, end);
    }

    public static BezierCurve curve(Pose... points) {
        return new BezierCurve(points);
    }

    public enum Poses {
        CLOSE_START(110, 132, 0),
        CLOSE_SCORE(95, 90, 0),
        CLOSE_END(120, 100, 0),

        FAR_START(84, 8.5, 90),
        FAR_SCORE(84, 16, 0),
        FAR_END(108, 8.5, 0),

        R1_START(Artifacts.X_START, Artifacts.Y_1, Artifacts.HEADING),
        R1_END  (Artifacts.X_1_END, Artifacts.Y_1, Artifacts.HEADING),
        R2_START(Artifacts.X_START, Artifacts.Y_2, Artifacts.HEADING),
        R2_END  (Artifacts.X_2_END, Artifacts.Y_2, Artifacts.HEADING),
        R3_START(Artifacts.X_START, Artifacts.Y_3, Artifacts.HEADING),
        R3_END  (Artifacts.X_3_END, Artifacts.Y_3, Artifacts.HEADING),

        INTAKE_ROW(Artifacts.HEADING),

        DRIVE_OFFSET(180),

        GATE_INTAKE(129.5, 59.5, 30),

        CORNER_SPIKE(132, 9, 0),

        PARK(38, 32.75, 90),

        GOAL(135, 135), // Todo: Make goal pose 138, 138 when we retune for new hood
        ;

        public final double x;
        public final double y;
        public final double heading;

        Poses(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        Poses(double x, double y) {
            this.x = x;
            this.y = y;
            this.heading = 0;
        }

        Poses(double heading) {
            this.x = 0;
            this.y = 0;
            this.heading = heading;
        }

        Poses() {
            this.x = 0;
            this.y = 0;
            this.heading = 0;
        }

        private static class Artifacts {
            // Todo: Don't artificially offset locations
            public static double Y_1 = 82.5 - 1.0;
            public static double Y_2 = 59.0 - 3.5;
            public static double Y_3 = 35.5 - 5.0;

            public static double X_START = 101;

            public static double X_1_END = 125 + 2;
            public static double X_2_END = 131;
            public static double X_3_END = 131;

            public static double HEADING = 0;
        }
    }

    // Zone things

    public static class Zones {
        public static final PolygonZone CLOSE_LAUNCH = new PolygonZone(
                new Point(0,     141.5),
                new Point(141.5, 141.5),
                new Point(70.75, 70.75)
        );

        public static final PolygonZone FAR_LAUNCH = new PolygonZone(
                new Point(41.17, 0),
                new Point(94.33, 0),
                new Point(70.75, 23.58)
        );

        private static final PolygonZone robotZone = new PolygonZone(16, 16);
    }

    public static boolean isInLaunchZone() {
        return Zones.robotZone.isInside(Zones.CLOSE_LAUNCH) || Zones.robotZone.isInside(Zones.FAR_LAUNCH);
    }

    public static boolean isNearLaunchZone() {
        double tol = 8;
        return distToLaunchZone() < tol;
    }

    public static double distToLaunchZone() {
        return Math.min(Zones.robotZone.distanceTo(Zones.CLOSE_LAUNCH), Zones.robotZone.distanceTo(Zones.FAR_LAUNCH));
    }

    public static void updateRobotLocation(Pose robotPose) {
        Zones.robotZone.setPosition(robotPose.getX(), robotPose.getY());
        Zones.robotZone.setRotation(robotPose.getHeading());

        savedPose = robotPose;
    }

    // Misc util methods

    public static double distToGoal() {
        return pose(Poses.GOAL).distanceFrom(savedPose);
    }
}
