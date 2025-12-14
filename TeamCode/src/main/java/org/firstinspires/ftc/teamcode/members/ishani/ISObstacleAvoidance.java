package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - Obstacle Avoidance ", group = "ISSensor")
public class ISObstacleAvoidance extends LinearOpMode {

    //  YOUR ROBOT PARTS 
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DistanceSensor frontSensor;
    private IMU imu;

    //  SETTINGS  
    private static final double DRIVE_POWER          = 0.2;
    private static final double MAX_TRAVEL_INCHES    = 84.0;  // 7 feet max
    private static final double SAFE_DISTANCE_INCHES = 18.0;  // Turn if closer
    private static final double TURN_DEGREES         = 90.0;
    private static final double COUNTS_PER_INCH      = 28.0;  // Need to update this.

    private double totalTraveled = 0.0;

    @Override
    public void runOpMode() {

        // 1. CONNECT MOTORS
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // 2. RESET ENCODERS
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  3. CONNECT DISTANCE SENSOR
        frontSensor = hardwareMap.get(DistanceSensor.class, "distance");

        //  4. CONNECT IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
        imu.resetYaw();


        // 5. UPDATE TELEMETRY
        telemetry.addData("OBSTACLE AVOIDANCE FIXED", "Power 0.2 | Max 7 feet");
        telemetry.addData("Safe distance", "%.0f inches", SAFE_DISTANCE_INCHES);
        telemetry.addData("Turn degrees", "%.0f°", TURN_DEGREES);
        telemetry.addData("Press PLAY → let's go safely!", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && totalTraveled < MAX_TRAVEL_INCHES) {

            double distance = frontSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance Ahead", "%.1f inches", distance);
            telemetry.addData("Total Traveled", "%.1f / %.0f inches", totalTraveled, MAX_TRAVEL_INCHES);

            //  STOP IF MAX TRAVELED INCHES REACHED!
            if (totalTraveled >= MAX_TRAVEL_INCHES) {
                stopDriving();
                telemetry.addData("MAX REACHED", "Stopping!");
                telemetry.update();
                break;
            }

            //  OBSTACLE DETECTED!
            if (distance < SAFE_DISTANCE_INCHES) {
                stopDriving();
                telemetry.addData("OBSTACLE!", "Turning %.0f°...", TURN_DEGREES);
                telemetry.update();

                turnDegrees(TURN_DEGREES);  // Turn away
                driveForwardInches(12, DRIVE_POWER);  // Move forward a bit to clear
                totalTraveled += 12;  // Add to total
                // If you want to turn back, add: turnDegrees(-TURN_DEGREES);
            }

            //  IF NO OBSTACLE, PATH CLEAR,  DRIVE FORWARD
            else {
                // NEW: Drive a small step (1 inch) accurately, then check again
                driveForwardInches(1, DRIVE_POWER);
                totalTraveled += 1;
                telemetry.addData("CLEAR", "Driving forward 1 inch at a time");
            }

            telemetry.addData("Heading", "%.1f°", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            sleep(100);  // Small pause to not rush
        }

        stopDriving();
        telemetry.addData("DONE", "Traveled %.1f inches safely! Great job!", totalTraveled);
        telemetry.update();
    }

    //  HELPER FUNCTION: DRIVE FORWARD EXACT INCHES USING ENCODERS
    private void driveForwardInches(double inches, double power) {
        int target = (int) (inches * COUNTS_PER_INCH);  // Calculate encoder target

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set targets
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);

        // Switch to RUN_TO_POSITION mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        // Wait until done (check if busy)
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Driving", "To %.0f inches...", inches);
            telemetry.update();
        }

        stopDriving();

        // Back to RUN_USING_ENCODER for next time
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // HELPER FUNCTION: TURN EXACT DEGREES (IMU)
    private void turnDegrees(double degrees) {

        // Step 1: Figure out where we want to end up
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = currentHeading + degrees;

        // Step 2: How far off are we right now? or error correction
        double error = targetHeading - currentHeading;

        // Step 3: Fix the "circle problem" - always turn the short way
        if (error > 180) {
            error = error - 360;
        } else if (error < -180) {
            error = error + 360;
        }

        // Step 4: Keep turning until we're close enough (within 3 degrees is good!)
        while (opModeIsActive() && Math.abs(error) > 3.0) {

            double turnPower = 0.25;  //  medium speed when turning

            // Decide which way to turn
            if (error > 0) {
                // Need to turn LEFT
                frontLeft.setPower(-turnPower);
                backLeft.setPower(-turnPower);
                frontRight.setPower(turnPower);
                backRight.setPower(turnPower);
            } else {
                // Need to turn RIGHT
                frontLeft.setPower(turnPower);
                backLeft.setPower(turnPower);
                frontRight.setPower(-turnPower);
                backRight.setPower(-turnPower);
            }

            // Check again how far we are from the target
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = targetHeading - currentHeading;

            // Fix the circle problem again
            if (error > 180) {
                error = error - 360;
            } else if (error < -180) {
                error = error + 360;
            }
        }

        // Step 5: Stop the motors when we're done
        stopDriving();
    }

    //  HELPER: STOP ALL MOTORS 
    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}