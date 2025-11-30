package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class MovingForwardByXInches extends LinearOpMode {

    final double pulsesPerRevolution = 537.7;
    final double wheelDiameterInches = 4;
    final double countsPerInch = pulsesPerRevolution / wheelDiameterInches * Math.PI;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontRightDrive;
    DcMotor frontLeftDrive;
    DcMotor rearRightDrive;
    DcMotor rearLeftDrive;

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;



        newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches*countsPerInch);
        newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches*countsPerInch);
        newRearLeftTarget = rearLeftDrive.getCurrentPosition() + (int)(leftInches*countsPerInch);
        newRearRightTarget = rearRightDrive.getCurrentPosition() + (int)(rightInches*countsPerInch);

        telemetry.addData("ready","done");
        telemetry.update();
        sleep(1000);
            /*frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Ser Zero Pwer Behaiour Completed","done2");
            telemetry.update();*/
        try {
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("ZeroPower","Ready");
            telemetry.update();

            sleep(1000);

            frontRightDrive.setTargetPosition(newFrontRightTarget);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            rearRightDrive.setTargetPosition(newRearRightTarget);
            rearLeftDrive.setTargetPosition(newRearLeftTarget);

            telemetry.addData("ready","TargetPosition");
            telemetry.update();
            sleep(1000);

            frontRightDrive.setPower(Math.abs(speed));
            frontLeftDrive.setPower(Math.abs(speed));
            rearRightDrive.setPower(Math.abs(speed));
            rearLeftDrive.setPower(Math.abs(speed));

            telemetry.addData("ready","Speed");
            telemetry.update();
            sleep(1000);



            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("RunMode set to ", "RunToposition");
            telemetry.update();
            sleep(1000);


            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy() && frontLeftDrive.isBusy())) {

                telemetry.addData("Moving,","Forward");
                telemetry.update();
            }
            frontRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            rearRightDrive.setPower(0);
            rearLeftDrive.setPower(0);

            telemetry.addData("powered","down");
            telemetry.update();

            sleep(250);
        }
        catch (Exception e)
        {
            telemetry.addData("Exception",e.getMessage());
            telemetry.update();
            sleep(10000);

        }




    }




    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        if(opModeIsActive()) {
            encoderDrive(0.2, 10, 10, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            telemetry.addData("moving","forward");
            telemetry.update();
            encoderDrive(0.2, -10, -10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
            telemetry.addData("moving","back");
            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
}

