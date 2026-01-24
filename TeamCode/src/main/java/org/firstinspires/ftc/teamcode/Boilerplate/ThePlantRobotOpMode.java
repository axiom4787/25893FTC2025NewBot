package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class ThePlantRobotOpMode extends LinearOpMode {
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public DcMotor intake, shooter;
    public CRServo turretLeft, turretRight;
    public Servo linearActuator;
    public HuskyLens huskyLens;
    public IMU imu;

    Config config = new Config();

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    final public void runOpMode() {
        internalInitElectronics();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        opModeInit();

        waitForStart();
        runtime.reset();

        opModeRunOnce();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            opModeRunLoop();

            telemetry.update();
        }
    }

    public HuskyLens.Block getTargetBlock() {
        HuskyLens.Block target = null;
        HuskyLens.Block[] blocks = huskyLens.blocks();
//        telemetry.addData("Block count", blocks.length);
        double biggestBlockSize = 0f;
        for (HuskyLens.Block block : blocks) {
            if (block.height * block.width > biggestBlockSize) {
                target = block;
                biggestBlockSize = block.height * block.width;
            }
//            telemetry.addData("Block", block.toString());
        }

        return target;
    }
    public static double calculateTurret(HuskyLens.Block target) {
        double base = -((target.x / 160f) - 1f) * 0.5f;
        return Math.signum(base) * Math.pow(Math.abs(base), 1.4f) * 2.5f;
    }
    public static double calculateHood(HuskyLens.Block target) {
        double base = (target.y - 110f) / 160f * 0.035f;
        return Math.signum(base) * Math.pow(Math.abs(base), 1.4f) * 12f;
    }
    public double[] getRealPos(HuskyLens.Block targetBlock, double REAL_WIDTH) {
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

    public abstract void opModeInit();
    public abstract void opModeRunOnce();
    public abstract void opModeRunLoop();

    private void internalInitElectronics() {
        config.init(hardwareMap);

        frontLeftDrive = config.frontLeftDrive;
        frontRightDrive = config.frontRightDrive;
        backLeftDrive = config.backLeftDrive;
        backRightDrive = config.backRightDrive;
        intake = config.intake;
        shooter = config.shooter;
        turretLeft = config.turretServoLeft;
        turretRight = config.turretServoRight;
        linearActuator = config.linearActuator;
        huskyLens = config.huskyLens;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        imu = config.imu;
    }
}
