package org.firstinspires.ftc.teamcode.Helper;

public class DecodeUtil {

    public static enum AutoType {
        BLUE_NEAR,
        BLUE_FAR,
        RED_NEAR,
        RED_FAR
    }

    public static String getAprilTagType(AutoType autoType){
        String aprilTagType = "";
        if (autoType == AutoType.BLUE_NEAR)
            aprilTagType = DecodeAprilTag.BLUE_APRIL_TAG;
        else if (autoType == AutoType.BLUE_FAR)
            aprilTagType = DecodeAprilTag.BLUE_APRIL_TAG;
        else if (autoType == AutoType.RED_NEAR)
            aprilTagType = DecodeAprilTag.RED_APRIL_TAG;
        else if (autoType == AutoType.RED_FAR)
            aprilTagType = DecodeAprilTag.RED_APRIL_TAG;

        return aprilTagType;

    }

    public static enum NearAutoStages {
        BACK_UP,
        SHOOT,
        GET_MORE_BALLS,
        GO_BACK_TO_SHOOTING_ZONE,
        SHOOT_AGAIN,
        MOVE_OUT_OF_SHOOTING_ZONE,
        END
    }

    public static enum FarAutoStages {
        MOVE_TO_SHOOTING_ZONE,
        SHOOT,
        MOVE_OUT_OF_SHOOTING_ZONE,
        END
    }
}
