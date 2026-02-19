package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class DriveSubsystem {
    Config config = new Config();
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {
        config.init(hardwareMap);

        frontLeftDrive = config.frontLeftDrive;
        frontRightDrive = config.frontRightDrive;
        backLeftDrive = config.backLeftDrive;
        backRightDrive = config.backRightDrive;

        imu = config.imu;
    }

    private void drive(double forward, double right, double rotate) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double frontRightPower = forward - right - rotate;
        double frontLeftPower  = forward + right + rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        // Normalize the values so no wheel power exceeds 100%
        double max = Math.abs(frontLeftPower);
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

//        telemetry.addData("Front left/Right drive", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//        telemetry.addData("Back  left/Right drive", "%4.2f, %4.2f", backLeftPower, backRightPower);
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert cartesian offset to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }
}
