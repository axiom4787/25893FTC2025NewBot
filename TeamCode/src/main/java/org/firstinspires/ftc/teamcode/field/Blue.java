
package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;
public class Blue {

    // Auto
    // Start Pose of our robot.
    public static final Pose START_POSE   = new Pose(64, 9, Math.toRadians(90));
    public static final Pose APRILTAG_POSE = new Pose(58,90, Math.toRadians(90));

    // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static final Pose SCORE_POSE   = new Pose(58, 90, Math.toRadians(135));
    // Highest (First Set) of Artifacts from the Spike Mark.
    public static final Pose ALIGN1_POSE  = new Pose(50, 84, Math.toRadians(0)); // og = 41.5, 84
    public static final Pose PICKUP1_POSE = new Pose(24, 84, Math.toRadians(0));

    // Middle (Second Set) of Artifacts from the Spike Mark.
    public static final Pose ALIGN2_POSE  = new Pose(50, 60, Math.toRadians(0)); // og = 41.5, 60
    public static final Pose PICKUP2_POSE = new Pose(24, 60, Math.toRadians(0));

    // Lowest (Last Set) of Artifacts from the Spike Mark.
    public static final Pose ALIGN3_POSE  = new Pose(50, 36, Math.toRadians(0)); // og = 41.5, 36
    public static final Pose PICKUP3_POSE = new Pose(24, 36, Math.toRadians(0));
    // Teleop
    // Gate Start & End
    public static final Pose GATE_START_POSE = new Pose(18, 72, Math.toRadians(0));
    public static final Pose GATE_END_POSE = new Pose(15, 72, Math.toRadians(0));

    // Endgame
    public static final Pose ENDGAME_POSE = new Pose(106, 33, Math.toRadians(90));

    private Blue() {}
}
