package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// This is your FULL AUTONOMOUS program!
// It drives forward, strafes left/right, and turns using encoders + IMU
@Autonomous(name = "IS - Full Auto (FWD + STRAFE + TURN)", group = "ISDrive")
public class FullAutoDriveStrafeTurn extends LinearOpMode {

    // === YOUR ROBOT PARTS ===
    private DcMotor frontLeft, frontRight, backLeft, backRight;  // the 4 wheels
    private IMU imu;  // the "compass" sensor for perfect turns

    // === YOUR MAGIC NUMBERS FROM CALIBRATION (change these to yours!) ===
    private static final double FORWARD_TICKS_PER_INCH = 48.7;   // from forward calibrator
    private static final double STRAFE_TICKS_PER_INCH  = 52.3;   // from strafe calibrator (usually a bit higher)

    @Override
    public void runOpMode() {

        // ———————— STEP 1: CONNECT THE MOTORS ————————
        // These lines "find" your motors by their names in the phone config
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Reverse the left motors so "forward" works for all
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ———————— STEP 2: CONNECT THE IMU SENSOR ————————
        // This tells the IMU how your Control Hub is mounted on the robot
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,     // change if your logo points sideways
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD  // change if USB points sideways
                )
        ));

        // Show on phone that everything is ready
        telemetry.addData("FULL AUTO READY", "Drives forward, strafes, turns!");
        telemetry.addData("Keep robot STILL", "Press INIT → Wait green light → Press PLAY");
        telemetry.update();

        // Reset the IMU heading to 0° when you press INIT
        imu.resetYaw();

        // Wait for you to press PLAY on the phone
        waitForStart();

        // ———————— STEP 3: THE AUTONOMOUS PATH ————————
        // Now the robot does this sequence automatically:
        // 1. Drive forward 24 inches
        // 2. Strafe right 12 inches
        // 3. Turn left 90°
        // 4. Drive forward 24 inches again
        // 5. Strafe left 12 inches
        // 6. Turn right 90° back to start direction

        telemetry.addData("STARTING PATH", "Forward → Strafe right → Turn left → Forward → Strafe left → Turn right");
        telemetry.update();

        driveForward(24);    // forward 24 inches
        strafeRight(12);     // strafe right 12 inches
        turnLeft(90);        // turn left 90°
        driveForward(24);    // forward another 24 inches
        strafeLeft(12);      // strafe left 12 inches
        turnRight(90);       // turn right 90° (back to original direction)

        // All done!
        stopDriving();
        telemetry.addData("AUTO COMPLETE", "Robot back to start — great job!");
        telemetry.update();
    }

    // ———————— HELPER FUNCTION: DRIVE FORWARD EXACT DISTANCE ————————
    // Uses encoders to go exactly the inches you want
    private void driveForward(double inches) {
        int ticks = (int)(inches * FORWARD_TICKS_PER_INCH);  // convert inches to encoder "ticks"

        resetEncoders();  // start encoders at 0
        setTargetPosition(ticks, ticks, ticks, ticks);  // all wheels go forward
        runToPosition();  // tell motors to go to that position

        // Start moving at 60% speed
        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        // Wait until done + show progress on phone
        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Driving Forward", "%.1f / %.0f inches",
                    frontLeft.getCurrentPosition() / FORWARD_TICKS_PER_INCH, inches);
            telemetry.addData("Heading", "%.1f°", getHeading());
            telemetry.update();
        }

        stopDriving();  // stop when done
        runUsingEncoders();  // back to normal mode
        telemetry.addData("FORWARD DONE", "%.0f inches", inches);
        telemetry.update();
        sleep(300);  // short pause
    }

    // ———————— HELPER FUNCTION: STRAFE RIGHT EXACT DISTANCE ————————
    // Uses encoders for perfect sideways move (positive inches = right)
    private void strafeRight(double inches) {
        strafe(inches);
    }

    // ———————— HELPER FUNCTION: STRAFE LEFT EXACT DISTANCE ————————
    // Just negative inches for left
    private void strafeLeft(double inches) {
        strafe(-inches);
    }

    // Main strafe function (positive = right, negative = left)
    private void strafe(double inches) {
        int ticks = (int)(inches * STRAFE_TICKS_PER_INCH);

        resetEncoders();
        setTargetPosition(ticks, -ticks, -ticks, ticks);  // opposite directions for strafe
        runToPosition();

        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Strafing", inches > 0 ? "RIGHT →" : "LEFT ←");
            telemetry.addData("Distance", "%.1f / %.0f inches",
                    Math.abs(frontLeft.getCurrentPosition()) / STRAFE_TICKS_PER_INCH, Math.abs(inches));
            telemetry.addData("Heading", "%.1f°", getHeading());
            telemetry.update();
        }

        stopDriving();
        runUsingEncoders();
        telemetry.addData("STRAFE DONE", "%.0f inches", inches);
        telemetry.update();
        sleep(300);
    }

    // ———————— HELPER FUNCTION: TURN LEFT EXACT DEGREES ————————
    // Uses IMU for perfect turn (no drift!)
    private void turnLeft(double degrees) {
        double target = getHeading() + degrees;
        turnToHeading(target);
    }

    // ———————— HELPER FUNCTION: TURN RIGHT EXACT DEGREES ————————
    private void turnRight(double degrees) {
        double target = getHeading() - degrees;
        turnToHeading(target);
    }

    // Main turn function (goes to exact heading)
    private void turnToHeading(double target) {
        double error = target - getHeading();
        while (error > 180) error -= 360;   // always shortest way
        while (error <= -180) error += 360;

        while (opModeIsActive() && Math.abs(error) > 1.0) {  // stop when close enough
            double power = error > 0 ? 0.5 : -0.5;  // left or right
            power = Math.max(0.3, Math.abs(power));  // not too slow

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            error = target - getHeading();
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;

            telemetry.addData("Turning to", "%.0f°", target);
            telemetry.addData("Current", "%.1f°  Error: %.1f°", getHeading(), error);
            telemetry.update();
        }

        stopDriving();
        telemetry.addData("TURN DONE", "At %.1f°", getHeading());
        telemetry.update();
        sleep(300);  // short pause
    }

    // ———————— SUPER HELPER FUNCTIONS (don't change these) ————————
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