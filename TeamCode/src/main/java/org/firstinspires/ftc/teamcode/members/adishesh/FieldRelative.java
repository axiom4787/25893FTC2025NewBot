package org.firstinspires.ftc.teamcode.members.adishesh;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Adi's field relative")
public class FieldRelative extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double OVERSHOOT = 1.036; //over shoot per inch
    static final double COUNTS_PER_DEGREE = 11.06; // 8400/360
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;

    private double powerMultiplier = 0.25;
    private IMU imu;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);


        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        while (opModeIsActive()) {
            move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


    }
    private void move(double axial, double lateral, double yaw) {
        double totalAngle = Math.atan2(axial, lateral);
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
        double theta = totalAngle - currentYaw;
        double radius = Math.hypot(axial,lateral);

        double newAxial = radius * Math.sin(theta);
        double newLateral = radius * Math.cos(theta);

        double frontLeftPower = newAxial+newLateral+yaw;
        double frontRightPower = newAxial-newLateral-yaw;
        double rearLeftPower = newAxial-newLateral+yaw;
        double rearRightPower = newAxial+newLateral-yaw;

      /*  double max = Math.max(frontLeftPower, frontRightPower);
        max = Math.max(max, rearLeftPower);
        max = Math.max(max, rearRightPower);
        if (max > 1) {
            max = 1;
        }
*/


        frontLeftDrive.setPower(frontLeftPower * powerMultiplier);
        frontRightDrive.setPower(frontRightPower * powerMultiplier);
        rearLeftDrive.setPower(rearLeftPower* powerMultiplier);
        rearRightDrive.setPower(rearRightPower * powerMultiplier);
    }

}