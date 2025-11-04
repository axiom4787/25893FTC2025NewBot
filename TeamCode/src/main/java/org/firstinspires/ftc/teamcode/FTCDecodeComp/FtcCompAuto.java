
package org.firstinspires.ftc.teamcode.FTCDecodeComp;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "FTC Comp", group = "Ftc Comp")
@Disabled
//This is to not duplicate the TeleOp
//we need to add the DcMotors
public class FtcCompAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;// = hardwareMap.dcMotor.get("frontLeft");
    private DcMotor backLeftMotor;// = hardwareMap.dcMotor.get("backLeft");
    private DcMotor frontRightMotor;// = hardwareMap.dcMotor.get("frontRight");
    private DcMotor backRightMotor;// = hardwareMap.dcMotor.get("backRight");
    private DcMotorEx outtake;
    private CRServo intake;
    private CRServo storageWheel;
    private AprilTagProcessor aprilTag;
    public static final String WEBCAM_NAME = "Webcam 1";
    public static final int GOAL_TAG_ID = 24;
    double position = 0.312;
    private VisionPortal visionPortal;


    int targetY = 1050;
    int targetX = 1025;

    int targetZ = 1000;
    int targetA = 1350;
    int curTarget = 0;
    double Minrange = 0;
    double Maxrange = 0;
    double range = 0.05;

    double distanceToTarget = 0;
    double turnToTarget = 0;

    double targetVelocity = .6;
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT = 0;


//    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
//    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
//    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
//    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
//    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
//    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;
//
//    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
//    final double INTAKE_COLLECT = -1.0;
//    final double INTAKE_OFF = 0.0;
//    final double INTAKE_DEPOSIT = 0.5;
//
//    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
//    final double WRIST_FOLDED_IN = 0.8333;
//    final double WRIST_FOLDED_OUT = 0.5;
//
//    /* A number in degrees that the triggers can adjust the arm position by */
//    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
//
//    /* Variables that are used to set the arm to a specific position */
//    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
//    double armPositionFudgeFactor;
//    int HoldPosition;


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        CRServo intake = hardwareMap.crservo.get("intake");
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        CRServo storageWheel = hardwareMap.crservo.get("storageWheel");

        //setting direction for motors


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        position = 0;//remove
        double addposition = 0.001;
        double addedcurrentPosition = position;
        double subposition = -0.001;
        double subcurrentPosition = position;
        int armPosition = 0;
        double armMovePos = 0.001;
        boolean gravityCondition = true;


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeTagProcessor();

        //waiting for start
        waitForStart();
        //  elbow.setTargetPosition(0);
        //  elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //  elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //reseting variable called runtime
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            turnToTarget = getAngleToTag(GOAL_TAG_ID );
            distanceToTarget = getDistanceToTag(GOAL_TAG_ID );
            telemetry.addData("At distance to long range shoot",distanceToTarget);
            telemetry.addData("At angle to shoot",turnToTarget);
            if (gamepad1.left_trigger > 0) {
                if (distanceToTarget > 125 && distanceToTarget < 129){
                    curTarget = targetA;

                }
                if (distanceToTarget < 70 && distanceToTarget > 65){
                    curTarget = targetY;
                }
                if (distanceToTarget < 65 && distanceToTarget > 60){
                    curTarget = targetX;
                }
                if (distanceToTarget < 60 && distanceToTarget > 55){
                    curTarget = targetZ;
                }
                if (turnToTarget < -1 || turnToTarget > 1) {
                    if (turnToTarget < -1) {
                        rx = 0.3;
                    }
                    else {
                        rx = -0.3;
                    }
                }
            }

            // Denominator is the l
            //
            //
            //
            // largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double speedModifier = Math.abs(1.5);
            denominator *= speedModifier;

            if (gamepad1.y) {
                denominator *= 2.5;
            }

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);




            runtime.reset();
            if (gamepad1.dpad_down){
                intake.setPower (-1);
            }
            if (gamepad1.dpad_up){
                intake.setPower(1);
            }
            if (gamepad1.a) {
                curTarget = targetA;
                telemetry.addData("motorTargetSetTo1", curTarget);
            }
//.5 425
            if (gamepad1.b) {
                curTarget = 0;
                telemetry.addData("motorTargetSetTo12", curTarget);

            }
            if (gamepad1.y) {
                curTarget = targetY;
                telemetry.addData("motorTargetSetTo123", curTarget);
            }

            if (gamepad1.right_bumper){
                curTarget = 0;
            }

            telemetry.addData("motorTargetSetTo", curTarget);
            double currentVelocity = outtake.getVelocity();
            Minrange = curTarget -(curTarget* range);
            Maxrange = curTarget +(curTarget * range);
            if (currentVelocity < Minrange) {
                double currentPowerMore = outtake.getPower();
                currentPowerMore = currentPowerMore + 0.001;
                outtake.setPower(currentPowerMore);
                telemetry.addData("Works!", 0);


            }
            if (currentVelocity > Maxrange) {
                double currentPowerLess = outtake.getPower();
                currentPowerLess = currentPowerLess - 0.001;
                outtake.setPower(currentPowerLess);
                telemetry.addData("Works1234!", 0);
            }
            if (currentVelocity < Maxrange && currentVelocity > Minrange) {
                runtime.reset();
                int timeToSpeed = (int) runtime.milliseconds();
                while (timeToSpeed <2500){
                    storageWheel.setPower(-1);
                    timeToSpeed = (int) runtime.milliseconds();
                }
                storageWheel.setPower(0);
                if (timeToSpeed > 2500) {
                    storageWheel.setPower(0);
                }

                telemetry.addData("SHOOTNOW!", 0);

            }

            if (gamepad1.x){
                intake.setPower(0);
            }

            telemetry.update();

            if (gamepad1.left_bumper){
                curTarget = 600;
                storageWheel.setPower(0);

            }





        }



    }
    private void displayTagInfo() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        int countTagsFound = currentDetections.size();

        if (countTagsFound == 0) {
            telemetry.addLine("Looking for tags");
            telemetry.addLine(String.format("Cannot Detect Goal Tag id %d", GOAL_TAG_ID));
        } else {
            for (AprilTagDetection detection : currentDetections) {
                int id = detection.id;
                if (id == GOAL_TAG_ID) {
                    telemetry.addLine(String.format("Range (inches)\t %6.1f", detection.ftcPose.range));
                    telemetry.addLine(String.format("Bearing (deg) \t %6.1f", detection.ftcPose.bearing));
                    telemetry.addLine("Tag Position:");
                    telemetry.addLine("\tPositive is Left of center");
                    telemetry.addLine("\tNegative is Right of center");
                    telemetry.addLine(String.format("Tag Name: %s \t", detection.metadata.name));
                    telemetry.addLine(String.format("Tag ID: %d \t", detection.metadata.id));
                    telemetry.addData("Total tag count in view", countTagsFound);
                }
            }
        }

    }
    private int getDistanceToTag(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int range = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                range = (int) detection.ftcPose.range;
                break;
            }
        }
        return range;
    }

    private int getAngleToTag(int tagID) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int angle = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagID) {
                angle = (int) detection.ftcPose.bearing;
                break;
            }
        }
        return angle;
    }
    private void initializeTagProcessor() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));

        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(aprilTag, true);
    }

}