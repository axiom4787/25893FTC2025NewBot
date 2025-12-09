package org.firstinspires.ftc.teamcode.members.ishani;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@Autonomous(name="Basic Mecanum Sequence", group="Linear OpMode")
@Autonomous
public class SelfCorrect extends LinearOpMode {

    private  IMU imu ;
    // Declare OpMode members for each of the 4 motors.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    final double COUNTS_PER_REVOLUTION = 537.7;
    final double WHEEL_DIAMETER_INCHES = 4.09;
    final double countsPerInch = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * Math.PI);
    //final double countsPerDegree = COUNTS_PER_REVOLUTION/45;
    final double countsPerDegree = 11.06;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        // These motor names must correspond to the names configured in the robot configuration file.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");

        // Set motor directions.
        // You may need to reverse motors depending on your physical setup.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).
        waitForStart();

        if (opModeIsActive()) {

            moveForward(48);

            telemetry.addData("Status", "Done");
            telemetry.update();
        }
    }

    private void moveForward(double distance) {
        int countsToMove= (int)( distance * countsPerInch);
        applyEncoder(countsToMove, countsToMove,countsToMove, countsToMove,0.1);
    }

    private void moveBackward(double distance) {
        int countsToMove= (int)( distance * countsPerInch*-1);
        applyEncoder(countsToMove, countsToMove,countsToMove, countsToMove,0.2);
    }

    private void rotate(int degrees) {
        int countsToMove = (int) countsPerDegree*degrees;
        applyEncoder(countsToMove, -countsToMove, countsToMove, -countsToMove, 0.2);
    }
    private void applyEncoder(int frontLeftTarget, int frontRightTarget, int rearLeftTarget, int rearRightTarget, double power) {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        /*
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        frontLeftDrive.setTargetPosition(frontLeftTarget);
        frontRightDrive.setTargetPosition(frontRightTarget);
        backLeftDrive.setTargetPosition(rearLeftTarget);
        backRightDrive.setTargetPosition(rearRightTarget);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);

        //imu.resetYaw();


        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        while(frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                backLeftDrive.isBusy() && backRightDrive.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            // Get the current heading (yaw angle)

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

            // Calculate the deviation (error) from the target heading
            double targetHeading = 0;
            double headingError = targetHeading - currentHeading;

            // Apply proportional correction: larger error means larger correction
            double HEADING_GAIN = 0.01;
            headingError = headingError * HEADING_GAIN;


            // Adjust motor powers based on the correction
            double frontLeftPower = power - headingError;
            double rearLeftPower = power - headingError;
            double frontRightPower = power + headingError;
            double rearRightPower = power + headingError;

            // Clip motor powers to stay within the valid range (-0.3 to 0.3)
            frontLeftPower = Range.clip(frontLeftPower, -0.3, 0.3);
            frontRightPower = Range.clip(frontRightPower, -0.3, 0.3);
            rearLeftPower = Range.clip(rearLeftPower, -0.3, 0.3);
            rearRightPower = Range.clip(rearRightPower, -0.3, 0.3);

            // Set motor powers
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(rearLeftPower);
            backRightDrive.setPower(rearRightPower);

            // Add telemetry for monitoring
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Correction", headingError);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Rear Left Power", rearLeftPower);
            telemetry.addData("Rear Right Power", rearRightPower);

            telemetry.update();
        }

        // Stop all motors after the timeout
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
