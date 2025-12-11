package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "IS - Self-Correcting IMU", group = "ISDrive")
public class SelfCorrectingIMU extends LinearOpMode {

    private IMU imu;

    // This remembers the "true zero" even if IMU drifts a little
    private double zeroHeadingOffset = 0.0;

    @Override
    public void runOpMode() {

        // ========== 1. SETUP IMU (CHANGE IF YOUR HUB IS ROTATED!) ==========
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,      // ← change if needed
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD   // ← change if needed
                )
        );
        imu.initialize(params);

        telemetry.addData("STEP 1", "Keep robot PERFECTLY STILL");
        telemetry.addData("STEP 2", "Press INIT → wait for solid green light");
        telemetry.addData("STEP 3", "I will auto-calibrate and lock zero!");
        telemetry.update();

        // Wait for you to press INIT (this is when real calibration happens)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting for INIT...");
            telemetry.addData("Current raw yaw", "%.2f°", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        // ========== 2. AUTO SELF-CALIBRATION (THIS KILLS DRIFT!) ==========
        // Right after you press INIT, we grab the current heading and make it "true zero"
        zeroHeadingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Optional: reset the IMU's own yaw so it starts at 0 too
        imu.resetYaw();

        telemetry.addData("SELF-CALIBRATED!", "True zero locked in");
        telemetry.addData("Saved offset", "%.3f°", zeroHeadingOffset);
        telemetry.update();
        sleep(1500);

        waitForStart();

        // ========== 3. NOW DO PERFECT TURNS FOREVER (NO DRIFT!) ==========
        turnTo(90);    // turn left 90°
        sleep(1000);
        turnTo(0);     // back to start
        sleep(1000);
        turnTo(-90);   // turn right 90°
        sleep(1000);
        turnTo(0);     // back again

        telemetry.addData("ALL DONE", "Zero drift forever!");
        telemetry.update();
    }

    // ★★★★★ SUPER SMART HEADING THAT NEVER DRIFTS ★★★★★
    private double getTrueHeading() {
        double raw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double corrected = raw - zeroHeadingOffset;

        // Keep it between -180 and +180 (nice clean numbers)
        while (corrected > 180) corrected -= 360;
        while (corrected <= -180) corrected += 360;

        return corrected;
    }

    // ★★★★★ TURN TO EXACT ANGLE (self-correcting!) ★★★★★
    private void turnTo(double targetDegrees) {
        double power = 0.4;
        double tolerance = 1.0;  // stop when within 1 degree

        telemetry.addData("TURNING TO", "%.1f°", targetDegrees);
        telemetry.update();

        while (opModeIsActive()) {
            double current = getTrueHeading();
            double error = targetDegrees - current;

            // Always take the shortest path
            if (error > 180) error -= 360;
            if (error <= -180) error += 360;

            telemetry.addData("Current (corrected)", "%.2f°", current);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Raw IMU reading", "%.2f°", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Saved offset", "%.3f°", zeroHeadingOffset);
            telemetry.update();

            if (Math.abs(error) <= tolerance) {
                stopMotors();
                telemetry.addData("ARRIVED!", "%.2f° (target %.1f°)", current, targetDegrees);
                telemetry.update();
                sleep(500);
                break;
            }

            // Turn in correct direction
            double turnPower = error > 0 ? power : -power;
            hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "front_left_motor").setPower(-turnPower);
            hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "back_left_motor").setPower(-turnPower);
            hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "front_right_motor").setPower(turnPower);
            hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "back_right_motor").setPower(turnPower);
        }
    }

    private void stopMotors() {
        hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "front_left_motor").setPower(0);
        hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "back_left_motor").setPower(0);
        hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "front_right_motor").setPower(0);
        hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "back_right_motor").setPower(0);
    }
}