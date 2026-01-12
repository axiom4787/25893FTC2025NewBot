package org.firstinspires.ftc.teamcode.members.bala;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous

public class DriveStraight extends LinearOpMode {



    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // 5203 Series Yellow -Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    static final double     COUNTS_PER_DEGREE       = 11.06;




    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition());


        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // S1: Forward 12 Inches with 10 Sec timeout
        encoderDrive(DRIVE_SPEED,  84,  84, 0, 40);
        encoderDrive(TURN_SPEED,    0,  0, 90, 10);
        //encoderDrive(DRIVE_SPEED,  60,  60, 0, 40);
        //encoderDrive(TURN_SPEED,    0,  0, 90, 10);
        //encoderDrive(DRIVE_SPEED,  84,  84, 0, 40);
        //encoderDrive(TURN_SPEED, 0, 0, 90, 10);
        //encoderDrive(DRIVE_SPEED, 60, 60, 0, 40);  // S3: Reverse 24 Inches with 4 Sec timeout





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double degrees, double timeoutS) {


        int newFrontLeftTarget;
        int newFrontRightTarget;


        int newBackLeftTarget;
        int newBackRightTarget;



        // If the robot should rotate
        if(degrees!=0) {
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
        }
        else //If the robot should move front or back
        {
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        }




        // Ensure that the OpMode is still active
        if (opModeIsActive()) {


            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);


            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            IMU imu = hardwareMap.get(IMU.class, "imu");
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next st
            //
            //ep, use (isBusy() || isBusy()) in the loop test.
            imu.resetYaw();
            double backRightPower = 0;
            double backLeftPower = 0;
            double frontRightPower = 0;
            double frontLeftPower = 0;
            double currentHeading = 0;
            double targetHeading = 0;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy()
                            && backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                currentHeading = orientation.getYaw(AngleUnit.DEGREES);

                  targetHeading = 0;
                double headingError = targetHeading - currentHeading;

                double heading_GAIN = 0.01;

                headingError = headingError * heading_GAIN;

                double power = 0.5;

                 frontLeftPower = power + headingError;
                 frontRightPower = power - headingError;
                 backLeftPower = power - headingError;
                 backRightPower = power + headingError;


                // Display it for the driver.
                telemetry.addData("Running to", " FrontLeft: %7d FrontRight:%7d  BackLeft: %7d BackRight:%7d",
                        newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", " FrontLeft: %7d FrontRight:%7d  BackLeft: %7d BackRight:%7d",
                        frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(),
                        backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);

            telemetry.update();


            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}











