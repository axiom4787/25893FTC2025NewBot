package org.firstinspires.ftc.teamcode.adikan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Adi's auto Rotation")
public class RotationForMe extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;// eg: TETRIX Motor Encoder
    static final double COUNTS_PER_DEGREE = 23.34 ; // 8400/360
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

       //move(8400,8400 , 0.2);// 360
      //  move(4200, 4200,0.2);// 180
      //  move(2100,2100,0.2);//90
       // move(6300,6300,0.2);//270

        rotate(360, 1);
       // sleep(1000);
       // move(-5,-5,0.2);
     //   sleep(1000);




    }

    private void move(int leftDist, int rightDist, double power) {

        telemetry.addData("Starting method move","");
        telemetry.update();
        sleep(2000);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + leftDist;
        int newFrontRightPosition = frontRightDrive.getCurrentPosition() - rightDist;
        int newRearLeftPosition = rearLeftDrive.getCurrentPosition() + leftDist  ;
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() - rightDist;


        frontLeftDrive.setTargetPosition(newFrontLeftPosition);
        frontRightDrive.setTargetPosition(newFrontRightPosition);
        rearLeftDrive.setTargetPosition(newRearLeftPosition);
        rearRightDrive.setTargetPosition(newRearRightPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        while(opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {

        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);



        telemetry.addData("Moved to ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();

    }

    private void rotate(int degrees, double power) {

        telemetry.addData("Starting method move", "");
        telemetry.update();
        sleep(2000);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
        int newFrontRightPosition = frontRightDrive.getCurrentPosition() + (int) (- degrees * COUNTS_PER_DEGREE);
        int newRearLeftPosition = rearLeftDrive.getCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() + (int) (- degrees * COUNTS_PER_DEGREE);


        frontLeftDrive.setTargetPosition(newFrontLeftPosition);
        frontRightDrive.setTargetPosition(newFrontRightPosition);
        rearLeftDrive.setTargetPosition(newRearLeftPosition);
        rearRightDrive.setTargetPosition(newRearRightPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        while (opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {

        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);


        telemetry.addData("Moved to ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();
    }


}
