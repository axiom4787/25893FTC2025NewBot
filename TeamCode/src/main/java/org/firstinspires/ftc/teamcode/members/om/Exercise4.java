package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Om - Exercise 4", group="Exercises")
public class Exercise4 extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        rearRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y; // Y-stick value is negative for forward
            double turn  = -gamepad1.right_stick_x; // X-stick value is negative for left

            if (gamepad1.a) {
                double temp = drive;
                drive = turn;
                turn = temp;
                telemetry.addData("Mode", "Crazy Mode (X and Y swapped)");
            } else {
                telemetry.addData("Mode", "Normal Mode");
            }


            double forwardSpeedMultiplier;
            if (gamepad1.b) {
                forwardSpeedMultiplier = 1.0;
            } else {
                forwardSpeedMultiplier = 0.5;
            }

            drive *= forwardSpeedMultiplier;

            double leftPower    = drive + turn;
            double rightPower   = drive - turn;

            leftPower  = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);

            frontLeft.setPower(leftPower);
            rearLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            rearRight.setPower(rightPower);

            telemetry.addData("Forward Speed Multiplier", forwardSpeedMultiplier);
            telemetry.addData("Left Motor Power", leftPower);
            telemetry.addData("Right Motor Power", rightPower);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}