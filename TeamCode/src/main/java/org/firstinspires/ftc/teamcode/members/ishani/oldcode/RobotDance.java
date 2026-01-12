package org.firstinspires.ftc.teamcode.members.ishani.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.

 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous

public class RobotDance extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;


    @Override
    public void runOpMode() {
        // get motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        // correct flipped motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            dance(10);
        }
    }

    private void dance(int times) {
        for (int i = 0; i < times; i++)
        {
            // move forward for 3 seconds
            move(1, 0, 0, 0.1);
            sleep(5000);
            // move backward for 3 seconds
            move(-1, 0, 0, 0.1);
            sleep(5000);
            // move clockwise for 2 seconds
            move(0, 0, 1, 2);
            sleep(5000);
            // move anti-clockwise for 2 seconds
            move(0, 0, -1, 2);
            sleep(5000);
            // move left for 3 seconds
            move(0, -1, 0, 0.1);
            sleep(5000);
            // move right for 3 seconds
            move(0, 1, 0, 0.1);
            sleep(5000);
        }

        stop();
    }

    private void move(float axial, float lateral, float yaw, double power) {
        // calculate motor power for each wheel
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double rearLeftPower = axial - lateral + yaw;
        double rearRightPower = axial + lateral - yaw;

        frontLeftPower *= power;
        frontRightPower *= power;
        rearLeftPower *= power;
        rearRightPower *= power;

        // cap motor powers to -1 and -1
        if (frontLeftPower < 0) {
            frontLeftPower = Math.max(-1, frontLeftPower);
        } else {
            frontLeftPower = Math.min(1, frontLeftPower);
        }

        if (frontRightPower < 0) {
            frontRightPower = Math.max(-1, frontRightPower);
        } else {
            frontRightPower = Math.min(1, frontRightPower);
        }

        if (rearLeftPower < 0) {
            rearLeftPower = Math.max(-1, rearLeftPower);
        } else {
            rearLeftPower = Math.min(1, rearLeftPower);
        }

        if (rearRightPower < 0) {
            rearRightPower = Math.max(-1, rearRightPower);
        } else {
            rearRightPower = Math.min(1, rearRightPower);
        }

        // run each motor
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        rearLeftDrive.setPower(rearLeftPower);
        rearRightDrive.setPower(rearRightPower);
    }

    private void stopMoving() {
        move(0, 0, 0, 0);
    }
}
