package org.firstinspires.ftc.teamcode.members.ishani.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
//setting motors as private
public class FirstEncoder extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    //Calculate the counts per inch by
    // doing countspermotor* drivegearreduction/ wheeldiameterinches*PI
    // define the variable as double
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // 5203 Series Yellow -Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    double DRIVE_SPEED = 0.1;

    @Override
    public void runOpMode() {
        //hardware.map basically retrieves the left motor and right motor
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");

        telemetry.addData("Starting at", "%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition());

        telemetry.update();
        waitForStart();


        encoderDrive(DRIVE_SPEED, 120, 120, 10.0);
        encoderDrive(DRIVE_SPEED, -120, -120, 10.0);
      
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;

        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {

            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);


            frontRightDrive.setTargetPosition(newFrontRightTarget);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);


            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy()
                            && backLeftDrive.isBusy() && backRightDrive.isBusy())) {
                telemetry.addData("Running to", " FrontLeft: %7d FrontRight: %7d BackLeft: %7d BackRight: %7d,",
                        newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", "FrontLeft: %7d FrontRight: %7d BackLeft: %7d BackRight: %7d",
                frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(),
                        backRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition());
                telemetry.update();
            }

            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);


            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);


        }


    }


}
