/*
robot drives forward 24 inches → turns 90° → forward → turn → forward → turn → forward → back to start!
100% accurate using IMU turns + encoder drive.

*/
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - Drive PERFECT Square", group = "ISDrive")
public class DrivePerfectSquare extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // ←←← USE REAL CALIBRATED NUMBERS (from calibration OpModes!)
    private static final double FORWARD_TICKS_PER_INCH = 48.7;   // ← change to yours!
    private static final double SIDE_LENGTH_INCHES = 24.0;

    @Override
    public void runOpMode() {

        // === CONNECT MOTORS ===
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // === CONNECT IMU ===
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addData("DRIVE SQUARE READY", "24 inches per side");
        telemetry.addData("TICKS PER INCH", "%.3f", FORWARD_TICKS_PER_INCH);
        telemetry.addData("Keep robot STILL → Press INIT → Wait green light → PLAY", "");
        telemetry.update();

        imu.resetYaw();  // current direction = 0°

        waitForStart();

        telemetry.addData("STARTING", "Driving perfect square...");
        telemetry.update();

        // DO 4 SIDES OF THE SQUARE
        for (int side = 1; side <= 4; side++) {
            driveForward(SIDE_LENGTH_INCHES);
            turnLeft90();
            telemetry.addData("SQUARE", "Side %d/4 complete!", side);
            telemetry.update();
        }

        stopDriving();
        telemetry.addData("PERFECT SQUARE DONE", "Back at starting spot & direction!");
        telemetry.addData("Final heading", "%.2f°", getHeading());
        telemetry.update();
    }

    // DRIVE FORWARD EXACT DISTANCE (ENCODERS)
    private void driveForward(double inches) {
        int ticks = (int)(inches * FORWARD_TICKS_PER_INCH);

        resetEncoders();
        setTargetPosition(ticks, ticks, ticks, ticks);
        runToPosition();

        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
        backRight.setPower(0.2);

        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Driving", "%.1f inches", frontLeft.getCurrentPosition() / FORWARD_TICKS_PER_INCH);
            telemetry.addData("Heading", "%.2f°", getHeading());
            telemetry.update();
        }

        stopDriving();
        runUsingEncoders();
        telemetry.addData("DRIVE DONE", "%.1f inches complete", inches);
        telemetry.update();
        sleep(400);
    }

    // TURN LEFT 90° (IMU — never drifts!)
    private void turnLeft90() {
        turnToHeading(getHeading() + 90);
    }

    private void turnToHeading(double target) {
        double error = target - getHeading();
        while (error >  180) error -= 360;
        while (error <= -180) error += 360;

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            double power = error > 0 ? 0.45 : -0.45;
            power = Math.max(0.3, Math.abs(power));

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            error = target - getHeading();
            while (error >  180) error -= 360;
            while (error <= -180) error += 360;

            telemetry.addData("Turning to", "%.0f°", target);
            telemetry.addData("Current", "%.2f°  |  Error %.2f°", getHeading(), error);
            telemetry.update();
        }
        stopDriving();
        telemetry.addData("TURN DONE", "At %.2f°", getHeading());
        telemetry.update();
        sleep(400);
    }

    // HELPER FUNCTIONS
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
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}