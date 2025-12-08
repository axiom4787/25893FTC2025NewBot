package org.firstinspires.ftc.teamcode.members.prathika;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class Drivestraight extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // 5203 Series Yellow -Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");  // Make sure it's named "imu" in config!


        // === 2. Tell IMU how the hub is mounted ===
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,     // Logo faces UP
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT     // USB faces LEFT
        );

        // Initialize IMU with correct orientation
        imu.initialize(new IMU.Parameters(orientation));



        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        waitForStart();

        driveStraight(0.1, 50);
    }
    public void driveStraight(double power, double distance) {
        int counts = (int) (distance * COUNTS_PER_INCH);

        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        while (opModeIsActive()) {
            if (frontLeftDrive.getCurrentPosition() >= counts || frontRightDrive.getCurrentPosition() >= counts || backLeftDrive.getCurrentPosition() >= counts || backRightDrive.getCurrentPosition() >= counts) break;

            double error = 0 - getHeading();
            double correction = error * 0.01;

            moveVector(power, 0, correction);
        }

        moveVector(0,0,0);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return -orientation.getYaw(AngleUnit.DEGREES);
    }
    public void moveVector( double axial, double lateral, double yaw) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backRightPower = axial + lateral - yaw;
        double backLeftPower = axial - lateral + yaw;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}


