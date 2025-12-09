package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Simple IMU Tilt & Speed", group = "Test")
public class SimpleIMU extends LinearOpMode {

    // Declare IMU
    private IMU imu;

    @Override
    public void runOpMode() {

        // === 1. Get the IMU from robot config ===
        imu = hardwareMap.get(IMU.class, "imu");  // Make sure it's named "imu" in config!

        // === 2. Tell IMU how the hub is mounted ===
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,     // Our logo faces up
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT     // Our USB faces left
        );

        // Initialize IMU with correct orientation
        imu.initialize(new IMU.Parameters(orientation));

        // Optional: Reset yaw to 0 at start
        imu.resetYaw();

        telemetry.addData("Status", "IMU Ready! Move the robot to see values.");
        telemetry.update();

        // Wait for driver to press PLAY
        waitForStart();

        // === 3. Main loop: Keep reading and showing values ===
        while (opModeIsActive()) {

            // Get current angles (Yaw, Pitch, Roll)
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            // Get rotation speeds
            AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


            // === 4. Print everything
            telemetry.addData("=== ROTATION (Angles) ===", "");
            telemetry.addData("Yaw (Heading)", "%.1f°", angles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (Front/Back Tilt)", "%.1f°", angles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Left/Right Tilt)", "%.1f°", angles.getRoll(AngleUnit.DEGREES));

            telemetry.addData("=== SPEED (Velocity) ===", "");
            telemetry.addData("Yaw Speed (Turning)", "%.1f °/sec", velocity.zRotationRate);
            telemetry.addData("Pitch Speed", "%.1f °/sec", velocity.xRotationRate);
            telemetry.addData("Roll Speed", "%.1f °/sec", velocity.yRotationRate);

            // Press Y to reset heading to 0
            if (gamepad1.y) {
                imu.resetYaw();
                telemetry.addData("Yaw", "Reset to 0!");
            }

            telemetry.update();  // Send to phone
        }
    }
}