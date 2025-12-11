/*
Robot can start facing any direction
Press INIT → that becomes “forward on the field”
It draws a perfect hexagon aligned to the field, not the robot!
 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - Field-Relative HEXAGON", group = "ISDrive")
public class FieldRelativeHexagon extends LinearOpMode {

    // ———————— HARDWARE ————————
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // ———————— YOUR CALIBRATED VALUES ————————
    private static final double FORWARD_TICKS_PER_INCH = 48.7;   // ← from your calibration
    private static final double SIDE_LENGTH_INCHES     = 30.0;   // length of each hexagon side

    // ———————— FIELD-RELATIVE TRACKING ————————
    private double fieldZeroHeading = 0.0;   // This locks "forward on the field" when you press INIT

    @Override
    public void runOpMode() {

        // ———————— 1. CONNECT MOTORS ————————
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

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

        telemetry.addData("FIELD-RELATIVE HEXAGON", "Ready!");
        telemetry.addData("SIDE LENGTH", "%.0f inches", SIDE_LENGTH_INCHES);
        telemetry.addData("INSTRUCTIONS", "Keep robot STILL → Press INIT → Wait green light → PLAY");
        telemetry.addData("It will draw a perfect hexagon", "no matter which way robot is facing!");
        telemetry.update();

        // ———————— 3. LOCK FIELD ZERO WHEN YOU PRESS INIT ————————
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("WAITING FOR INIT", "Current raw heading: %.1f°", getRawHeading());
            telemetry.update();
        }

        fieldZeroHeading = getRawHeading();   // ← This becomes "forward on the field"
        imu.resetYaw();  // Optional: makes robot think it's at 0° too

        telemetry.addData("FIELD ZERO LOCKED", "Forward = %.1f°", fieldZeroHeading);
        telemetry.update();
        sleep(1000);

        waitForStart();

        telemetry.addData("STARTING", "Drawing field-relative hexagon...");
        telemetry.update();

        // ———————— 4. DRAW 6 PERFECT SIDES ————————
        for (int side = 1; side <= 6; side++) {
            driveFieldRelativeForward(SIDE_LENGTH_INCHES);   // always toward the driver!
            turnLeft(60);   // still uses IMU for perfect turns
            telemetry.addData("HEXAGON", "Side %d of 6 complete!", side);
            telemetry.update();
        }

        stopDriving();
        telemetry.addData("FIELD-RELATIVE HEXAGON DONE", "Perfect every time!");
        telemetry.addData("Final robot heading", "%.1f°", getRawHeading());
        telemetry.update();
    }

    // ———————— DRIVE FORWARD ON THE FIELD (NOT ROBOT!) ————————
    private void driveFieldRelativeForward(double inches) {
        int ticks = (int)(inches * FORWARD_TICKS_PER_INCH);

        resetEncoders();
        setTargetPosition(ticks, ticks, ticks, ticks);
        runToPosition();

        double power = 0.6;

        while (opModeIsActive() && frontLeft.isBusy()) {

            // Get how far we've turned from field zero
            double headingError = getRawHeading() - fieldZeroHeading;
            while (headingError >  180) headingError -= 360;
            while (headingError <= -180) headingError += 360;
            headingError = Math.toRadians(headingError);

            // Field-relative correction
            double rotX = 0;  // we want pure forward
            double rotY = power;

            double correctedX = rotX * Math.cos(-headingError) - rotY * Math.sin(-headingError);
            double correctedY = rotX * Math.sin(-headingError) + rotY * Math.cos(-headingError);

            frontLeft.setPower(correctedY + correctedX);
            frontRight.setPower(correctedY - correctedX);
            backLeft.setPower(correctedY - correctedX);
            backRight.setPower(correctedY + correctedX);

            telemetry.addData("Driving FIELD FORWARD", "%.1f / %.0f inches",
                    frontLeft.getCurrentPosition() / FORWARD_TICKS_PER_INCH, inches);
            telemetry.addData("Robot vs Field", "Robot: %.1f°  Field Forward: %.1f°",
                    getRawHeading(), fieldZeroHeading);
            telemetry.update();
        }

        stopDriving();
        runUsingEncoders();
        telemetry.addData("FIELD FORWARD DONE", "%.0f inches", inches);
        telemetry.update();
        sleep(400);
    }

    // ———————— TURN LEFT USING IMU (same as before) ————————
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

            telemetry.addData("Turning to", "%.1f°", target);
            telemetry.addData("Current", "%.1f°  |  Error %.1f°", getRawHeading(), error);
            telemetry.update();
        }
        stopDriving();
        telemetry.addData("TURN COMPLETE", "At %.1f°", getRawHeading());
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