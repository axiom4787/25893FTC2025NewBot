package org.firstinspires.ftc.teamcode.members.adishesh;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "Adi's corrected movement")
public class MoveWithCorrection extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double OVERSHOOT = 1.036; //over shoot per inch
    static final double COUNTS_PER_DEGREE = 11.06 ; // 8400/360
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;
    private IMU imu;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        imu = hardwareMap.get(IMU.class, "imu");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);


        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        move(60, true, 0.1);
        //rotate(30,0.1);
        //move(36, false, 0.2);
    }

    private void moveLinearly(int leftdist, int rightdist, double power) {

        telemetry.addData("Starting to Move Linearly", "");
        telemetry.update();

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + (int) (leftdist * COUNTS_PER_INCH);
        int newFrontRightPosition = frontRightDrive.getCurrentPosition() + (int) (rightdist * COUNTS_PER_INCH);
        int newRearLeftPosition = rearLeftDrive.getCurrentPosition() + (int) (leftdist * COUNTS_PER_INCH);
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() + (int) (rightdist * COUNTS_PER_INCH);


        frontLeftDrive.setTargetPosition(newFrontLeftPosition);
        frontRightDrive.setTargetPosition(newFrontRightPosition);
        rearLeftDrive.setTargetPosition(newRearLeftPosition);
        rearRightDrive.setTargetPosition(newRearRightPosition);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        while (opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {
            int targetYaw = 0;
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
            double error = targetYaw - currentYaw;
            if (leftdist < 0) {
                error *= -1;
            }
            double errorGain = 0.01 * error; // Apply correction gradually

            double leftPower = power - errorGain;
            double rightPower = power + errorGain;
            double max = Math.max(leftPower, rightPower);
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);


        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        telemetry.addData("Finished Moving Linearly", "");
        telemetry.update();

    }

    private void rotate(int degrees, double power) {
        imu.resetYaw();

        telemetry.addData("Starting Rotation", "");
        telemetry.update();
        //sleep(2000);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        int newFrontRightPosition = frontRightDrive.getCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
        int newRearLeftPosition = rearLeftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);

        telemetry.addData("Calculated new positions to be - ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();


        frontLeftDrive.setTargetPosition(newFrontLeftPosition);
        frontRightDrive.setTargetPosition(newFrontRightPosition);
        rearLeftDrive.setTargetPosition(newRearLeftPosition);
        rearRightDrive.setTargetPosition(newRearRightPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        while (opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {
            int targetYaw = degrees;
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
            double error = targetYaw - currentYaw;

            double errorGain = 0.01 * error; // Apply correction gradually

            double leftPower = power - errorGain;
            double rightPower = power + errorGain;
            double max = Math.max(leftPower, rightPower);
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);

        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);


        telemetry.addData("Moved to ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();
    }
    private void move(int dist, boolean axial, double power) {

        imu.resetYaw();

        telemetry.addData("Starting to Move Linearly", "");
        telemetry.update();

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int reqCounts = (int) ((dist / OVERSHOOT ) * COUNTS_PER_INCH );
        int newFrontRightPosition;
        int newRearLeftPosition;

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + reqCounts;
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() + reqCounts;
        if (axial) {
            newFrontRightPosition = frontRightDrive.getCurrentPosition() + reqCounts;
            newRearLeftPosition = rearLeftDrive.getCurrentPosition() + reqCounts;
        } else {
            newFrontRightPosition = frontRightDrive.getCurrentPosition() - reqCounts;
            newRearLeftPosition = rearLeftDrive.getCurrentPosition() - reqCounts;
        }

        frontLeftDrive.setTargetPosition(newFrontLeftPosition);
        frontRightDrive.setTargetPosition(newFrontRightPosition);
        rearLeftDrive.setTargetPosition(newRearLeftPosition);
        rearRightDrive.setTargetPosition(newRearRightPosition);

        telemetry.addData("Calculated new positions to be - ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        while (opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {
            int targetYaw = 0;
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
            double error = targetYaw - currentYaw;
            if (dist < 0) {
                error *= -1;
            }
            double errorGain = 0.01 * error; // Apply correction gradually

            double leftPower = power - errorGain;
            double rightPower = power + errorGain;
            double max = Math.max(leftPower, rightPower);
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);


        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        telemetry.addData("Finished Moving Linearly", "");
        telemetry.update();

    }
}

