package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class HuskyLensCalculator  {
    static Config config = new Config();
    HuskyLens huskyLens;
    public HuskyLensCalculator(HardwareMap hardwareMap) {
        config.init(hardwareMap);
        huskyLens = config.huskyLens;
    }
    public com.qualcomm.hardware.dfrobot.HuskyLens.Block getTargetBlock() {
        com.qualcomm.hardware.dfrobot.HuskyLens.Block target = null;
        com.qualcomm.hardware.dfrobot.HuskyLens.Block[] blocks = huskyLens.blocks();
        double biggestBlockSize = 0f;
        for (com.qualcomm.hardware.dfrobot.HuskyLens.Block block : blocks) {
            if (block.height * block.width > biggestBlockSize) {
                target = block;
                biggestBlockSize = block.height * block.width;
            }
        }

        return target;
    }

    public double calculateTurret(com.qualcomm.hardware.dfrobot.HuskyLens.Block target) {
        double base = -((target.x / 160f) - 1f) * 0.5f;
        return Math.signum(base) * Math.pow(Math.abs(base), 1.4f) * 2.5f;
    }

    public double calculateHood(com.qualcomm.hardware.dfrobot.HuskyLens.Block target) {
        double base = (target.y - 110f) / 160f * 0.035f;
        return Math.signum(base) * Math.pow(Math.abs(base), 1.4f) * 12f;
    }

    public double[] getRealPos(com.qualcomm.hardware.dfrobot.HuskyLens.Block targetBlock, double REAL_WIDTH) {
        double HALFCAMERAWIDTH = 320f / 2f;
        double HALFCAMERAHEIGHT = 180f / 2f;

        // Should be how much you need to multiply the distance calculation by to be accurate
        // You can find by running `TuneGetRealPos` and finding how much you need to multiply by to get the actual value, then multiply whichever of these you are tuning by that value to get your new value. These are the default values that produce +- 0.5 in of accuracy
        double ZSCALAR = 333.333; // How much you need to multiply the z distance calculation by to be accurate
        double XSCALAR = 0.555; // Same as above but for x distance
        double YSCALAR = 0.555; // """

        double targetBlockScale = Math.max(targetBlock.width, targetBlock.height); // our tag could be rotated, so we will take the largest of these to prevent errors

        double zDistance = (REAL_WIDTH / targetBlockScale) * ZSCALAR; // distance away in inches

        double xDistance = ((targetBlock.x - HALFCAMERAWIDTH) / HALFCAMERAWIDTH) * zDistance * XSCALAR; // the distance from the center in inches locally
        double yDistance = ((HALFCAMERAHEIGHT - targetBlock.y) / HALFCAMERAHEIGHT) * zDistance * YSCALAR;

        return new double[] {xDistance, yDistance, zDistance}; // Local x, y, and z, all in inches relative to the view
    }
}
