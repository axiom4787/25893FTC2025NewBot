/*
 - * ULTIMATE ROBOT CLASS v3 — FINAL VERSION (2025-2026)
 - * Everything you will EVER need in ONE file:
 - * - Perfect encoder drive + strafe
 - * - Perfect IMU turns
 - * - Distance sensor
 - * - Telemetry everywhere
 - * - Public getters + setDrivePower() → NO MORE PRIVATE ERRORS!
 - *
 - * HOW TO USE:
 - * robot = new Robot(hardwareMap, telemetry);
 - * robot.driveStraight(24);
 - * robot.turn(90);
 - * robot.setDrivePower(...) in TeleOp
 -
 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private DistanceSensor distanceSensor;
    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        telemetry.addData("ROBOT", "Loading from RobotConfig...");
        telemetry.update();

        // ——— MOTORS ———
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ——— IMU — uses static config ———
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RobotConfig.LOGO_FACING_DIR,
                        RobotConfig.USB_FACING_DIR
                )
        ));
        imu.resetYaw();

        // ——— DISTANCE SENSOR — uses config name ———
        distanceSensor = hardwareMap.get(DistanceSensor.class, RobotConfig.DISTANCE_SENSOR_NAME);

        telemetry.addData("ROBOT READY", "All settings from RobotConfig!");
        telemetry.addData("FWD", "%.1f | STRAFE %.1f | 90° %.0f",
                RobotConfig.FORWARD_TICKS_PER_INCH,
                RobotConfig.STRAFE_TICKS_PER_INCH,
                RobotConfig.TICKS_FOR_90_DEGREES);
        telemetry.update();
    }

    public void driveStraight(double inches) {
        int ticks = (int)(inches * RobotConfig.FORWARD_TICKS_PER_INCH);
        timer.reset();
        resetEncoders();
        setTargetPosition(ticks, ticks, ticks, ticks);
        runToPosition();
        setPowerAll(RobotConfig.DRIVE_SPEED);

        while (frontLeft.isBusy() && timer.seconds() < 8) {
            telemetry.addData("DRIVE", "%.1f / %.1f in",
                    frontLeft.getCurrentPosition() / RobotConfig.FORWARD_TICKS_PER_INCH, inches);
            telemetry.addData("Dist", "%.1f in", getDistanceInches());
            telemetry.update();
        }
        stopMotors();
        runUsingEncoders();
    }

    public void strafe(double inches) {
        int ticks = (int)(Math.abs(inches) * RobotConfig.STRAFE_TICKS_PER_INCH);
        timer.reset();
        resetEncoders();
        if (inches > 0) setTargetPosition(ticks, -ticks, -ticks, ticks);
        else           setTargetPosition(-ticks, ticks, ticks, -ticks);
        runToPosition();
        setPowerAll(RobotConfig.DRIVE_SPEED);

        while (frontLeft.isBusy() && timer.seconds() < 8) {
            telemetry.addData("STRAFE", "%.1f in %s", Math.abs(inches), inches > 0 ? "RIGHT" : "LEFT");
            telemetry.update();
        }
        stopMotors();
        runUsingEncoders();
    }

    public void turn(double degrees) {
        double target = getHeading() + degrees;
        double error = target - getHeading();
        while (Math.abs(error) > 180) error -= 360 * Math.signum(error);

        telemetry.addData("TURN", "%.0f° %s", Math.abs(degrees), degrees > 0 ? "LEFT" : "RIGHT");
        while (Math.abs(error) > 1.0) {
            double power = error > 0 ? RobotConfig.TURN_SPEED : -RobotConfig.TURN_SPEED;
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            error = target - getHeading();
            while (Math.abs(error) > 180) error -= 360 * Math.signum(error);

            telemetry.addData("→", "%.1f° (error %.1f°)", getHeading(), error);
            telemetry.update();
        }
        stopMotors();
    }

    public void stopAtDistance(double targetInches) {
        while (getDistanceInches() > targetInches + 1.0) {
            double speed = getDistanceInches() < targetInches + 10 ? 0.2 : 0.3;
            setPowerAll(speed);
            telemetry.addData("STOPPING", "%.1f → %.1f in", getDistanceInches(), targetInches);
            telemetry.update();
        }
        stopMotors();
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);   frontRight.setPower(fr);
        backLeft.setPower(bl);    backRight.setPower(br);
    }

    public double getDistanceInches() { return distanceSensor.getDistance(DistanceUnit.INCH); }
    public double getHeading()        { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }

    // ——— HELPER METHODS ———
    private void setPowerAll(double p) {
        frontLeft.setPower(p); frontRight.setPower(p);
        backLeft.setPower(p);  backRight.setPower(p);
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

    private void stopMotors() { setPowerAll(0); }
}