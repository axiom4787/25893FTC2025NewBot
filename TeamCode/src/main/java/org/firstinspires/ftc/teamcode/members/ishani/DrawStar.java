/*
the robot drives forward → turns 144° → forward → turn → repeat 5 times → draws a beautiful 5-point star on the floor!

FORWARD_TICKS_PER_INCH → use real calibrated number
STAR_LEG_LENGTH_INCHES → make it bigger (36-48) for a giant star!

A perfect 5-pointed star (★) is made of 5 straight lines that connect back to the start.
Here’s the magic math:

Full circle around the robot= 360°Number of points in a star= 5360° ÷ 5= 72°
BUT… if you only turned 72° each time you would just draw a regular pentagon (5-sided shape).
To make the lines skip over and create the cool star points, you actually turn two steps at a time instead of one.
So:
2 steps × 72° = 144°
That’s the secret!
Every time you go forward and turn 144° left (or right), the next line jumps over one point and connects perfectly to make the star.
Visual proof:
textStart → drive → turn 144°
      → drive → turn 144°
      → drive → turn 144°
      → drive → turn 144°
      → drive → turn 144° → back to start! ★
That’s why 144° is the magic star angle — it works every single time, no matter how big or small you make the star.
Fun fact: 144° is exactly 2/5 of a circle → that’s why it closes perfectly after 5 moves.

 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - Draw STAR Pattern", group = "ISDrive")
public class DrawStar extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // ←←← YOUR REAL NUMBERS (from your calibration OpModes!)
    private static final double FORWARD_TICKS_PER_INCH = 48.7;   // ← change to yours
    private static final double STAR_LEG_LENGTH_INCHES = 30.0;  // length of each star point

    @Override
    public void runOpMode() {

        // CONNECT MOTORS
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // CONNECT IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addData("STAR PATTERN READY", "30-inch legs");
        telemetry.addData("TICKS PER INCH", "%.2f", FORWARD_TICKS_PER_INCH);
        telemetry.addData("Keep robot STILL → Press INIT → Wait green → PLAY", "");
        telemetry.update();

        imu.resetYaw();  // current direction = 0°

        waitForStart();

        telemetry.addData("DRAWING STAR", "5 points, 144° turns...");
        telemetry.update();

        // DRAW 5-POINT STAR (magic number: turn 144° left each time!)
        for (int point = 1; point <= 5; point++) {
            driveForward(STAR_LEG_LENGTH_INCHES);
            turnLeft(144);  // ← 144° is the secret star angle!
            telemetry.addData("STAR", "Point %d of 5 complete!", point);
            telemetry.update();
        }

        stopDriving();
        telemetry.addData("★ STAR COMPLETE ★", "Beautiful 5-point star drawn!");
        telemetry.addData("Final heading", "%.2f°", getHeading());
        telemetry.update();
    }

    // DRIVE FORWARD EXACT DISTANCE
    private void driveForward(double inches) {
        int ticks = (int)(inches * FORWARD_TICKS_PER_INCH);

        resetEncoders();
        setTargetPosition(ticks, ticks, ticks, ticks);
        runToPosition();

        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Driving", "%.1f / %.0f inches",
                    frontLeft.getCurrentPosition() / FORWARD_TICKS_PER_INCH, inches);
            telemetry.addData("Heading", "%.1f°", getHeading());
            telemetry.update();
        }

        stopDriving();
        runUsingEncoders();
        telemetry.addData("DRIVE DONE", "%.0f inches", inches);
        telemetry.update();
        sleep(300);
    }

    // TURN LEFT BY EXACT DEGREES (IMU)
    private void turnLeft(double degrees) {
        double target = getHeading() + degrees;
        turnToHeading(target);
    }

    private void turnToHeading(double target) {
        double error = target - getHeading();
        while (error >  180) error -= 360;
        while (error <= -180) error += 360;

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            double power = error > 0 ? 0.5 : -0.5;
            power = Math.max(0.3, Math.abs(power));

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            error = target - getHeading();
            while (error >  180) error -= 360;
            while (error <= -180) error += 360;

            telemetry.addData("Turning →", "%.0f°", target);
            telemetry.addData("Current", "%.1f°  |  Error %.1f°", getHeading(), error);
            telemetry.update();
        }
        stopDriving();
        telemetry.addData("TURN DONE", "At %.1f°", getHeading());
        telemetry.update();
        sleep(300);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTargetPosition(int fl, int fr, int bl, int br) {
        frontLeft.setTargetPosition(fl);
        frontRight.setTargetPosition(fr);
        backLeft.setTargetPosition(bl);
        backRight.setTargetPosition(br);
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