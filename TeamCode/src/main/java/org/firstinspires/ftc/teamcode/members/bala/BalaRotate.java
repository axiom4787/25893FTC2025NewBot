package org.firstinspires.ftc.teamcode.members.bala;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



// hello

    @Autonomous

    public class BalaRotate extends LinearOpMode {

        // todo: write your code here

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

        @Override
        public void runOpMode() {
            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
            backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
            frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
            backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

System.out.println(COUNTS_PER_INCH * 196);
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
            //encoderDrive(DRIVE_SPEED,  12,  12, 10.0);  // S1: Forward 10 Inches with 5 Sec timeout
            // S2: Rotate 90 degrees clockwise (right turn)
            encoderDrive(TURN_SPEED, 196, -196, 1000  );
            // S2: Rotate 90 degrees counterclockwise (left turn)
            //Rotation Distance = pi * Track Width / 4(90 is a quarter of 360)
            //track width = around 13.5 inches
            //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            //encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


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
                                 double timeoutS) {
            int newFrontLeftTarget;
            int newFrontRightTarget;


            int newBackLeftTarget;
            int newBackRightTarget;

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

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

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next st
                //
                //ep, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (frontLeftDrive.isBusy() && frontRightDrive.isBusy()
                                && backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Running to",  " FrontLeft: %7d FrontRight:%7d  BackLeft: %7d BackRight:%7d",
                            newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                    telemetry.addData("Currently at",  " FrontLeft: %7d FrontRight:%7d  BackLeft: %7d BackRight:%7d",
                            frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(),
                            backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                    telemetry.update();
                }

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


