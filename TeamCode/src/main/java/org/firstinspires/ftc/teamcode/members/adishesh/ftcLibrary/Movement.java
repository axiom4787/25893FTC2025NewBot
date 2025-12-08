package org.firstinspires.ftc.teamcode.adikan.ftcLibrary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement {
    final double countsPerRevolution = 537.7;
    final double wheelDiameter = 4;
    final double countsPerInch = countsPerRevolution / (wheelDiameter * Math.PI);
    final double countsPerDegree = 11.06;

    HardwareMap hardwareMap;
    DcMotor frontLeftDrive;
    DcMotor rearLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearRightDrive;
    Telemetry telemetry;

    public Movement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_Right_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_motor");
        this.telemetry = telemetry;
    }

    private void move(int leftDist, int rightDist, double power) {

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newFrontLeftPosition = frontLeftDrive.getCurrentPosition() + (int) (leftDist * countsPerInch);
              int newFrontRightPosition = frontRightDrive.getCurrentPosition() + (int) (rightDist * countsPerInch);
        int newRearLeftPosition = rearLeftDrive.getCurrentPosition() + (int) (leftDist * countsPerInch);
        int newRearRightPosition = rearLeftDrive.getCurrentPosition() + (int) (rightDist * countsPerInch);


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

        while (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && rearLeftDrive.isBusy() && rearRightDrive.isBusy()) {

        }
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);


        telemetry.addData("Moved to ", "%7d %7d %7d %7d", newFrontLeftPosition, newFrontRightPosition, newRearLeftPosition, newRearRightPosition);
        telemetry.update();

    }
}
