package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldRelative")
public class FieldRelative extends OpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    @Override
    public void init() {
        // Motors — change these names only if yours are different
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Reverse left side (normal for mecanum)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // THE ONLY IMU SETUP THAT WORKS IN 2025-2026
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,        // ← change if your logo faces a different direction
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD     // ← change to LEFT, RIGHT, or BACKWARD if needed
        );

        imu.initialize(new IMU.Parameters(orientation));

        telemetry.addData("Status", "INITIALIZED – FIELD RELATIVE READY!");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y  = -gamepad1.left_stick_y;  // forward/back
        double x  = gamepad1.left_stick_x;   // strafe
        double rx = gamepad1.right_stick_x;  // turn

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.5;  // makes strafing feel great

        double fl = rotY + rotX + rx;
        double fr = rotY - rotX - rx;
        double bl = rotY - rotX + rx;
        double br = rotY + rotX - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);

        telemetry.addData("Drive", "Field-Relative ACTIVE");
        telemetry.addData("Heading", "%.1f°", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
}