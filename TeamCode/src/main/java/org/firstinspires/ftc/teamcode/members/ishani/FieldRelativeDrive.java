package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "IS - Field Relative Drive", group = "ISDrive")
public class FieldRelativeDrive extends LinearOpMode {

    // === MOTORS ===
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // === IMU (our compass) ===
    private IMU imu;

    @Override
    public void runOpMode() {

        // ———————— 1. CONNECT MOTORS ————————
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Reverse left side so pushing both sticks forward = robot goes forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ———————— 2. CONNECT & INITIALIZE IMU ————————
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // Reset yaw when you press INIT (this becomes "forward on the field")
        imu.resetYaw();

        telemetry.addData("FIELD RELATIVE READY", "Pro mode activated!");
        telemetry.addData("INSTRUCTIONS", "Keep robot still → Press INIT → Wait green light → Press PLAY");
        telemetry.addData("Left stick", "Moves robot on the FIELD (not robot direction!)");
        telemetry.addData("Right stick X", "Turns robot");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ———————— 3. READ GAMEPAD ————————
            double drive  = -gamepad1.left_stick_y;   // Forward/back on field
            double strafe = gamepad1.left_stick_x;    // Left/right on field
            double turn   = gamepad1.right_stick_x;   // Rotate robot

            // ———————— 4. GET CURRENT ROBOT ANGLE FROM IMU ————————
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // ———————— 5. CONVERT FIELD DIRECTIONS → ROBOT DIRECTIONS (THE MAGIC!) ————————
            // This math rotates your joystick inputs by the robot's current heading
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            // Make sure the robot doesn't go faster than 100% power
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
            double frontLeftPower  = (rotY + rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backLeftPower   = (rotY - rotX + turn) / denominator;
            double backRightPower  = (rotY + rotX - turn) / denominator;

            // ———————— 6. SEND POWER TO MOTORS ————————
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // ———————— 7. LIVE TELEMETRY (so you can see the magic) ————————
            telemetry.addData("MODE", "FIELD RELATIVE (Pro!)");
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(botHeading));
            telemetry.addData("Left Stick", "Drive %.2f  Strafe %.2f", drive, strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Motor Powers", "FL %.2f  FR %.2f  BL %.2f  BR %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}