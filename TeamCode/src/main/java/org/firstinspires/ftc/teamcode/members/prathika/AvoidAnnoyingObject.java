package org.firstinspires.ftc.teamcode.members.prathika;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class AvoidAnnoyingObject extends LinearOpMode {

    // Motors
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;


    private DistanceSensor distanceSensor;

    // Threshold in ICHESSSS

    static final double COUNTS_PER_DEGREE = 11.06;
    static final double THRESHOLD = 10.0;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // 5203 Series Yellow -Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {



        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRight");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // ---- Motor directions ----
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // ---- Encoders ----
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance (IN)", "%.1f", distance);
            telemetry.update();

            // If object detected
            if (distance < THRESHOLD) {

                stopMotors();
                avoidObject();

                break;

            } else {
                // Otherwise, drive straight
                driveStraightInches(0.2, 10);
            }

            idle();
        }

        stopMotors();
    }
    public void driveStraightInches(double power, double inches) {

        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        frontLeftDrive.setTargetPosition(moveCounts);
        frontRightDrive.setTargetPosition(moveCounts);
        backLeftDrive.setTargetPosition(moveCounts);
        backRightDrive.setTargetPosition(moveCounts);

        // Run to position
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        frontLeftDrive.setPower(Math.abs(power));
        frontRightDrive.setPower(Math.abs(power));
        backLeftDrive.setPower(Math.abs(power));
        backRightDrive.setPower(Math.abs(power));

        // Wait until all motors reach target
        while (opModeIsActive()
                && frontLeftDrive.isBusy()
                && frontRightDrive.isBusy()
                && backLeftDrive.isBusy()
                && backRightDrive.isBusy()) {

            telemetry.addData("Target", moveCounts);
            telemetry.addData("FL", frontLeftDrive.getCurrentPosition());
            telemetry.update();

            idle();
        }

        stopMotors();

        // Back to normal encoder mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void turnDegrees(double power, double degrees) {

        int moveCounts = (int) (degrees * COUNTS_PER_DEGREE);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        frontLeftDrive.setTargetPosition(moveCounts);
        backLeftDrive.setTargetPosition(moveCounts);
        frontRightDrive.setTargetPosition(-moveCounts);
        backRightDrive.setTargetPosition(-moveCounts);

        // Run to position
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        frontLeftDrive.setPower(Math.abs(power));
        frontRightDrive.setPower(Math.abs(power));
        backLeftDrive.setPower(Math.abs(power));
        backRightDrive.setPower(Math.abs(power));

        // Wait for motors to finish
        while (opModeIsActive()
                && frontLeftDrive.isBusy()
                && frontRightDrive.isBusy()
                && backLeftDrive.isBusy()
                && backRightDrive.isBusy()) {
            idle();
        }

        stopMotors();

        // Back to normal mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Stop all motors
    public void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    public void avoidObject() {

        frontLeftDrive.setPower(0.25);
        backLeftDrive.setPower(0.25);
        frontRightDrive.setPower(-0.25);
        backRightDrive.setPower(-0.25);
        sleep(600);


        turnDegrees(0.2,90 );
        driveStraightInches(0.1,24);
        turnDegrees(0.2,90 );
        driveStraightInches(0.1,24 );
        turnDegrees(0.2,-90);
        driveStraightInches(0.1,24);
        turnDegrees(0.2,90);
        sleep(800);

        stopMotors();
    }
}
