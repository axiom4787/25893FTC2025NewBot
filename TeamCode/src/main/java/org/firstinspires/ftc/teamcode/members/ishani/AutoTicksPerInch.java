/*
    How to Use It (takes 2 minutes)

    1. Put robot on floor
    2. Put a piece of tape exactly 36 inches in front of the robot
    3. Run this OpMode
    4. Press PLAY → robot drives forward slowly
    5. When the front of your robot touches the tape → PRESS THE SQUARE STOP BUTTON
    6. Look at your phone → it shows something like TICKS_PER_INCH = 47.8321
    7. Copy that number and paste it into all your other drive programs!
 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IS - Auto Find Ticks Per Inch", group = "ISDrive")
public class AutoTicksPerInch extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // You will physically measure this distance on the floor
    private final double DISTANCE_TO_DRIVE_INCHES = 36.0;  // 3 feet = easy to measure!

    @Override
    public void runOpMode() {

        // Connect motors
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(50);  // super fast updates!
        telemetry.addData("INSTRUCTIONS", "1. Put robot on floor");
        telemetry.addData("          ", "2. Put tape or mark exactly 36 inches in front");
        telemetry.addData("          ", "3. Line up front of robot with start line");
        telemetry.addData("PRESS PLAY →", "Robot will drive 36 inches and calculate for you!");
        telemetry.update();

        waitForStart();

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We will drive using time + low power so it doesn't overshoot
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start driving forward slowly
        frontLeft.setPower(0.4);
        frontRight.setPower(0.4);
        backLeft.setPower(0.4);
        backRight.setPower(0.4);

        telemetry.addData("DRIVING", "Go straight until you hit 36 inches...");
        telemetry.addData("Stop me with the STOP button when you reach the mark!", "");
        telemetry.update();

        // Wait for you to press STOP when the robot reaches the 36-inch mark
        while (opModeIsActive()) {
            int avgTicks = Math.abs(
                    (frontLeft.getCurrentPosition() +
                            frontRight.getCurrentPosition() +
                            backLeft.getCurrentPosition() +
                            backRight.getCurrentPosition()) / 4);

            telemetry.addData("Current average ticks", avgTicks);
            telemetry.addData("Live TICKS_PER_INCH", "%.3f", avgTicks / DISTANCE_TO_DRIVE_INCHES);
            telemetry.addData("INSTRUCTIONS", "When front of robot = 36 inches → PRESS SQUARE (STOP)");
            telemetry.update();
        }

        // Robot stopped! Grab the final numbers
        int finalTicks = Math.abs(
                (frontLeft.getCurrentPosition() +
                        frontRight.getCurrentPosition() +
                        backLeft.getCurrentPosition() +
                        backRight.getCurrentPosition()) / 4);

        double ticksPerInch = finalTicks / DISTANCE_TO_DRIVE_INCHES;

        // STOP MOTORS
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // FINAL RESULT — THIS IS YOUR MAGIC NUMBER!
        telemetry.clearAll();
        telemetry.setMsTransmissionInterval(100);
        telemetry.addData("CALIBRATION COMPLETE", "Copy this number!");
        telemetry.addData("TICKS_PER_INCH =", "%.4f", ticksPerInch);
        telemetry.addData("Use this in ALL your drive code!", "");
        telemetry.addData("Example:", "private final double TICKS_PER_INCH = %.4f;", ticksPerInch);
        telemetry.update();

        // Keep showing forever so you can screenshot or write it down
        while (opModeIsActive()) {
            sleep(1000);
        }
    }
}