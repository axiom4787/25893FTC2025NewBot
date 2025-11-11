package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Helper.Util;

public class Chassis {
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    RevHubOrientationOnRobot hubOrientation;
    double COUNTS_PER_MOTOR_REV = 537.7;
    double DRIVE_GEAR_REDUCTION = 1.0;
    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_INCH;

    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public IMU imu;

    private OpMode opMode;
    private DriveMode driveMode;

   private LinearOpMode linearOpMode;

    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public static enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public void init(OpMode opMode) {

        this.opMode = opMode;
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "backRightDrive");

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(hubOrientation));
        imu.resetYaw();

        linearOpMode = (LinearOpMode)opMode;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        //Make these values more accurate to make our heading more accurate
        odo.setOffsets(-4.5, 8, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();


    }

    public void setDriveMode(DriveMode driveMode) {
        // save driveMode to use in drive()
        this.driveMode = driveMode;

    }

    public void resetODOPosAndIMU() {
        odo.resetPosAndIMU();
    }

    public void updateTelemetry() {
        // opMode.telemetry.addData("Status", "Run Time: " + runtime);
        opMode.telemetry.addData("Drive Mode", driveMode);
        opMode.telemetry.addData("Front left/Right", JavaUtil.formatNumber(leftFrontPower, 4, 2) + ", " + JavaUtil.formatNumber(rightFrontPower, 4, 2));
        opMode.telemetry.addData("Back  left/Right", JavaUtil.formatNumber(leftBackPower, 4, 2) + ", " + JavaUtil.formatNumber(rightBackPower, 4, 2));
        odo.update();
        opMode.telemetry.addData("botHeading", JavaUtil.formatNumber(odo.getHeading(AngleUnit.RADIANS), 4, 2));
        opMode.telemetry.addData("botX", JavaUtil.formatNumber(odo.getPosX(DistanceUnit.CM), 4, 2));
        opMode.telemetry.addData("botY", JavaUtil.formatNumber(odo.getPosY(DistanceUnit.CM), 4, 2));
        opMode.telemetry.update();

    }

    /*
    public void Drive(double distance, double speed) {
        odo.update();

        double currentX = odo.getPosX(DistanceUnit.INCH);

        double error = distance-currentX;

        if (distance < 0) {
            speed*=-1;
        }

        //The error condition needs to be flipped if distance is negative
        //(Distance - current) will have different signs depending on sign of distance
        while ((distance < 0 && error <= 0) || (distance >= 0 && error >= 0)) {
            odo.update();
            currentX = odo.getPosX(DistanceUnit.INCH);
            error = distance - currentX;

            moveRobot(speed, 0, 0);
            opMode.telemetry.addData("CurrentPos", currentX);
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.update();
        }

        moveRobot(0,0,0);
    }

    public void Strafe(double distance, double speed) {
        odo.update();

        double currentY = odo.getPosY(DistanceUnit.INCH);

        double error = distance-currentY;

        if (distance < 0) {
            speed*=-1;
        }

        //The error condition needs to be flipped if distance is negative
        //(Distance - current) will have different signs depending on sign of distance
        while ((distance < 0 && error <= 0) || (distance >= 0 && error >= 0)) {
            odo.update();
            currentY = odo.getPosY(DistanceUnit.INCH);
            error = distance - currentY;

            moveRobot(0, speed, 0);
            opMode.telemetry.addData("CurrentPos", currentY);
            opMode.telemetry.addData("Error", error);
            opMode.telemetry.update();
        }

        moveRobot(0,0,0);
    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower = x + y + yaw;
        double frontRightPower = x - y - yaw;
        double backLeftPower = x - y + yaw;
        double backRightPower = x + y - yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

     */

    // Returns the current pose from odometry in mm and radians
    public Pose2D getPoseEstimate() {
        odo.update();
        double x = odo.getPosX(DistanceUnit.MM);
        double y = odo.getPosY(DistanceUnit.MM);
        double heading = odo.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }



    //Gyroscope
    public static double HEADING_THRESHOLD = 1.0;
    public static double P_TURN_COEFF = 0.03;


    public void printOdoTelemetry(){
        Util.printAllOdoTelemetry(odo, opMode.telemetry);
    }

    public void printIMUTelemetry(){
        Util.printIMUTelemetry(imu, opMode.telemetry);
    }

    public void turnToAngle(double targetAngle) {

        final double MIN_TURN_PWR = 0.2;
        final double MAX_TURN_PWR = 0.8;
        final double ERROR_RANGE_DEGREE = 3;

        while (((LinearOpMode) opMode).opModeIsActive()) {
            odo.update();
            double currentAngleRAD = odo.getHeading(AngleUnit.RADIANS);

            double currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(currentAngleRAD));
            double errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);


            opMode.telemetry.addData("Target Angle", targetAngle);
            opMode.telemetry.addData("Current Angle", currentAngle);
            opMode.telemetry.addData("Error Angle", errorAngle);
            opMode.telemetry.update();

            if( Math.abs(errorAngle) < ERROR_RANGE_DEGREE ) {
                Util.setMotorPower(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, 0, 0, 0);
                break;
            }

            //double turnPower = 0.01 * error;
            //turnPower = Math.max(-0.5, Math.min(turnPower, 0.5));

            double turnPower = ( (Math.abs(errorAngle) / 180) * (MAX_TURN_PWR - MIN_TURN_PWR) + MIN_TURN_PWR ) * Math.signum(errorAngle) * -1;

            //Half turn speed
            Util.setMotorPower(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, 0, 0, turnPower);
        }
    }








}
