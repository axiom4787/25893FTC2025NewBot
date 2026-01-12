package org.firstinspires.ftc.teamcode.members.adishesh;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Adi's Object Sensor")
public class AdiObjectSensor extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;
    private DistanceSensor distanceSensor;

    private int leftPos;
    private int rightPos;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        // forward 3, backward 3, clockwise 2, anticlockwise 2, left 3, right

        // Move front
        move(100, 100, 0.2);
        //sleep(1000);
        //Move back
        //move(-5,-5,0.2);
        //sleep(1000);




    }

    private void move(int leftDist, int rightDist, double power) {

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + (int) (leftDist * COUNTS_PER_INCH);
        int newFrontRightPosition = frontRightDrive.getCurrentPosition() + (int) (rightDist * COUNTS_PER_INCH);
        int newRearLeftPosition = rearLeftDrive.getCurrentPosition() + (int) (leftDist * COUNTS_PER_INCH);
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() + (int) (rightDist * COUNTS_PER_INCH);


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

        while(opModeIsActive() && frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                rearLeftDrive.isBusy() && rearRightDrive.isBusy() && distanceSensor.getDistance(DistanceUnit.INCH) > 20) {

            telemetry.addData("Distance of object in inches - ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);



        telemetry.addData("Moved to ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();

    }


}
