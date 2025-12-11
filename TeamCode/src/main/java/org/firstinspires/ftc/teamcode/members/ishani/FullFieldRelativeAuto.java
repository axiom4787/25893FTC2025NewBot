/*
driveFieldRelative(24, 0) → forward on field
driveFieldRelative(0, 24) → strafe right on field
driveFieldRelative(-18, 12) → diagonal on field!

This is the exact same system used in every single high-scoring Into the Deep autonomous
 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - FULL Field-Relative (FWD + STRAFE)", group = "ISDrive")
public class FullFieldRelativeAuto extends LinearOpMode {

    // ———————— HARDWARE ————————
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // ———————— YOUR REAL CALIBRATED VALUES ————————
    private static final double FORWARD_TICKS_PER_INCH = 48.7;   // from forward calibration
    private static final double STRAFE_TICKS_PER_INCH  = 52.3;   // from strafe calibration (usually higher!)

    // ———————— FIELD ZERO (locked when you press INIT) ————————
    private double fieldZeroHeading = 0.0;   // This defines "forward on the field"

    @Override
    public void runOpMode() {

        // ———————— 1. CONNECT MOTORS ————————
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ———————— 2. CONNECT IMU ————————
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addData("FULL FIELD-RELATIVE READY", "FWD + STRAFE + TURN");
        telemetry.addData("Keep robot STILL → Press INIT → Wait green → PLAY", "");
        telemetry.addData("Robot can start facing ANY direction!", "It will still draw perfect shapes");
        telemetry.update();

        // ———————— 3. LOCK FIELD ZERO WHEN YOU PRESS INIT ————————
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("WAITING", "Raw heading: %.1f°", getRawHeading());
            telemetry.update();
        }
        fieldZeroHeading = getRawHeading();   // ← This becomes "field forward"
        imu.resetYaw();

        telemetry.addData("FIELD ZERO LOCKED", "Forward = %.1f°", fieldZeroHeading);
        telemetry.update();
        sleep(1000);

        waitForStart();

        // ———————— 4. DEMO: PERFECT FIELD-RELATIVE MOVES ————————
        driveFieldRelative(24, 0);     // 24 inches forward on field
        driveFieldRelative(0, 24);     // 24 inches strafe right on field
        driveFieldRelative(-24, 0);    // 24 inches backward on field
        driveFieldRelative(0, -24);    // 24 inches strafe left on field

        turnLeft(90);                  // normal IMU turn
        driveFieldRelative(30, 0);     // go forward again (still field-relative!)

        stopDriving();
        telemetry.addData("DEMO COMPLETE", "Full field-relative control achieved!");
        telemetry.addData("Final heading", "%.1f°", getRawHeading());
        telemetry.update();
    }

    // ———————— FIELD-RELATIVE DRIVE: X = strafe, Y = forward ————————
    // Positive Y = forward on field, Positive X = right on field
    private void driveFieldRelative(double fieldY_inches, double fieldX_inches) {

        // Convert inches → encoder ticks (use correct value for direction!)
        int ticksY = (int)(fieldY_inches * FORWARD_TICKS_PER_INCH);
        int ticksX = (int)(fieldX_inches * STRAFE_TICKS_PER_INCH);

        // Reset and set up encoders
        resetEncoders();
        setTargetPosition(ticksY + ticksX, ticksY - ticksX, ticksY - ticksX, ticksY + ticksX);
        runToPosition();

        double basePower = 0.6;

        while (opModeIsActive() && frontLeft.isBusy()) {

            // Current heading error from field zero
            double headingError = Math.toRadians(getRawHeading() - fieldZeroHeading);

            // Desired field direction (Y = forward, X = strafe)
            double desiredX = fieldX_inches > 0 ? basePower : (fieldX_inches < 0 ? -basePower : 0);
            double desiredY = fieldY_inches > 0 ? basePower : (fieldY_inches < 0 ? -basePower : 0);

            // Rotate desired direction by current heading error
            double rotX = desiredX * Math.cos(-headingError) - desiredY * Math.sin(-headingError);
            double rotY = desiredX * Math.sin(-headingError) + desiredY * Math.cos(-headingError);

            // Apply to motors (mecanum math)
            frontLeft.setPower(rotY + rotX);
            frontRight.setPower(rotY - rotX);
            backLeft.setPower(rotY - rotX);
            backRight.setPower(rotY + rotX);

            // Live progress
            double doneY = frontLeft.getCurrentPosition() / FORWARD_TICKS_PER_INCH;
            double doneX = frontLeft.getCurrentPosition() / STRAFE_TICKS_PER_INCH;

            telemetry.addData("FIELD MOVE", "Y: %.1f  X: %.1f inches", doneY, doneX);
            telemetry.addData("Target", "Y: %.0f  X: %.0f", fieldY_inches, fieldX_inches);
            telemetry.addData("Robot Heading", "%.1f°", getRawHeading());
            telemetry.update();
        }

        stopDriving();
        runUsingEncoders();
        telemetry.addData("FIELD MOVE DONE", "Y=%.0f X=%.0f inches", fieldY_inches, fieldX_inches);
        telemetry.update();
        sleep(500);
    }

    // ———————— NORMAL IMU TURN (LEFT) ————————
    private void turnLeft(double degrees) {
        double target = getRawHeading() + degrees;
        turnToHeading(target);
    }

    private void turnToHeading(double target) {
        double error = target - getRawHeading();
        while (error >  180) error -= 360;
        while (error <= -180) error += 360;

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            double power = error > 0 ? 0.5 : -0.5;
            power = Math.max(0.3, Math.abs(power));

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            error = target - getRawHeading();
            while (error >  180) error -= 360;
            while (error <= -180) error += 360;

            telemetry.addData("Turning →", "%.1f°", target);
            telemetry.addData("Current", "%.1f°  Error: %.1f°", getRawHeading(), error);
            telemetry.update();
        }
        stopDriving();
        telemetry.addData("TURN DONE", "At %.1f°", getRawHeading());
        telemetry.update();
        sleep(400);
    }

    // ———————— HELPER FUNCTIONS ————————
    private double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTargetPosition(int fl, int fr, int bl, int br) {
        frontLeft.setTargetPosition(fl);   frontRight.setTargetPosition(fr);
        backLeft.setTargetPosition(bl);    backRight.setTargetPosition(br);
    }

    private void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runUsingEncoders() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopDriving() {
        frontLeft.setPower(0);  frontRight.setPower(0);
        backLeft.setPower(0);   backRight.setPower(0);
    }
}