package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - Draw HEXAGON", group = "ISDrive")
public class DrawHexagon extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // ←←← YOUR REAL CALIBRATED NUMBER!
    private static final double FORWARD_TICKS_PER_INCH = 48.7;   // ← change to yours
    private static final double SIDE_LENGTH_INCHES = 30.0;      // how long each side is

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

        telemetry.addData("HEXAGON READY", "%.0f-inch sides", SIDE_LENGTH_INCHES);
        telemetry.addData("Magic turn angle", "60° left each time");
        telemetry.addData("Keep robot STILL → Press INIT → Wait green → PLAY", "");
        telemetry.update();

        imu.resetYaw();  // current direction = 0°

        waitForStart();

        telemetry.addData("DRAWING HEXAGON", "6 perfect sides...");
        telemetry.update();

        // DRAW 6 SIDES OF HEXAGON
        for (int side = 1; side <= 6; side++) {
            driveForward(SIDE_LENGTH_INCHES);
            turnLeft(60);   // ← 60° is the magic hexagon angle!
            telemetry.addData("HEXAGON", "Side %d of 6 complete!", side);
            telemetry.update();
        }

        stopDriving();
        telemetry.addData("HEXAGON COMPLETE", "Perfect 6-sided shape drawn!");
        telemetry.addData("Final heading", "%.2f° (should be ~0°)", getHeading());
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

    // TURN LEFT EXACT DEGREES
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

            telemetry.addData("Turning to", "%.0f°", target);
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