package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IS - Encoder Drive Some Distance", group = "ISDrive")
public class Encoder extends LinearOpMode {

    // Your four drive motors (change names if yours are different!)
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // How many encoder ticks = 1 inch on YOUR robot?

    private final double TICKS_PER_INCH = 45;   // ←←← CHANGE THIS NUMBER

    @Override
    public void runOpMode() {

        // 1. Connect to motors
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // 2. Reverse left side so "forward" is forward for all motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Robot Ready! Press PLAY");
        telemetry.update();

        waitForStart();

        // 3. DRIVE 60 INCHES FORWARD!
        driveStraight(60, 0.2);   // 24 inches, 20% speed
//        driveStraight(-12, 0.5);  // backward 12"
        telemetry.addData(">", "Done! Robot drove 24 inches :)");
        telemetry.update();
    }

    // ★★★★★ THIS IS THE MAGIC FUNCTION ★★★★★
    public void driveStraight(double inches, double speed) {

        int ticks = (int)(inches * TICKS_PER_INCH);   // convert inches → encoder ticks

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tell motors where to go
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        // Turn on "go to position" mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving!
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Wait until all motors are done
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {
            // Show progress on phone
            telemetry.addData("Going to", "%d ticks (%d inches)", ticks, (int)inches);
            telemetry.addData("Current position", frontLeft.getCurrentPosition());
            telemetry.update();
        }

        // Stop!
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Turn off run-to-position mode (important for teleop later!)
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}