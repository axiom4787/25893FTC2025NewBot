package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "IS - Stop 10 Inches from Wall", group = "ISSensor")
public class Stop10InchesFromWall extends LinearOpMode {

    // ———————— YOUR ROBOT PARTS ————————
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DistanceSensor distanceSensor;   // ← your REV 2m sensor
    private IMU imu;                          // for keeping straight (optional but cool)

    // ———————— SETTINGS ————————
    private static final double TARGET_DISTANCE_INCHES = 10.0;   // stop when 10 inches away
    private static final double DRIVE_SPEED = 0.4;               // slow and safe
    private static final double FORWARD_TICKS_PER_INCH = 48.7;   // ← your calibrated number

    @Override
    public void runOpMode() {

        // ———————— 1. CONNECT MOTORS ————————
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ———————— 2. CONNECT DISTANCE SENSOR ————————
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");  // ← name in config!

        // ———————— 3. CONNECT IMU (to drive straight!) ————————
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        telemetry.addData("STOP AT 10 INCHES READY", "Point sensor forward!");
        telemetry.addData("Target", "%.0f inches from wall", TARGET_DISTANCE_INCHES);
        telemetry.addData("Keep robot STILL → Press INIT → Wait green → PLAY", "");
        telemetry.update();

        waitForStart();

        telemetry.addData("GO!", "Driving forward until 10 inches...");
        telemetry.update();

        // ———————— 4. DRIVE FORWARD UNTIL WE HIT 10 INCHES ————————
        while (opModeIsActive()) {

            double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);

            // Show live distance
            telemetry.addData("Current Distance", "%.1f inches", currentDistance);
            telemetry.addData("Target", "%.0f inches", TARGET_DISTANCE_INCHES);

            // ——— STOP WHEN WE ARE AT 10 INCHES (or closer) ———
            if (currentDistance <= TARGET_DISTANCE_INCHES) {
                stopDriving();
                telemetry.addData("PERFECT STOP!", "Exactly %.1f inches from wall!", currentDistance);
                telemetry.update();
                sleep(2000);  // celebrate for 2 seconds
                break;
            }

            // ——— KEEP DRIVING FORWARD (with IMU correction so we stay straight!) ———
            double headingError = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double correction = headingError * 0.02;   // small fix if we drift

            frontLeft.setPower(DRIVE_SPEED + correction);
            frontRight.setPower(DRIVE_SPEED - correction);
            backLeft.setPower(DRIVE_SPEED + correction);
            backRight.setPower(DRIVE_SPEED - correction);

            telemetry.addData("Driving", "Speed %.2f  |  Correction %.3f", DRIVE_SPEED, correction);
            telemetry.update();
        }

        // ———————— ALL DONE ————————
        stopDriving();
        telemetry.addData("MISSION COMPLETE", "Robot stopped perfectly at 10 inches!");
        telemetry.update();
    }

    // ———————— HELPER: STOP ALL MOTORS ————————
    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}