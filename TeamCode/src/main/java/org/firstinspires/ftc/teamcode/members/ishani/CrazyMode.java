package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IS - Crazy Mode Drive", group = "ISDrive")
public class CrazyMode extends LinearOpMode {

    // Declare your motors here
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {

        // CONNECT MOTORS
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Most robots need some motors reversed so they all move forward together
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        // frontRight and backRight stay FORWARD

        telemetry.addData("Status", "Robot Ready! Press Play :)");
        telemetry.update();

        waitForStart();  // Wait for the driver to press PLAY

        while (opModeIsActive()) {

            // 1. Get joystick values (remember: pushing stick UP gives NEGATIVE Y)
            double y = -gamepad1.left_stick_y;  // Forward / Back
            double x = gamepad1.left_stick_x;   // Strafe Left / Right
            double turn = gamepad1.right_stick_x; // Rotate

            double forward;
            double strafe;

            // 2. CRAZY MODE: Press A to swap forward/back with strafe!
            if (gamepad1.a) {
                forward = x;   // Left/right stick now moves robot forward/back
                strafe  = y;   // Forward/back stick now strafes
                telemetry.addData("MODE", "🤪 CRAZY MODE ON! 🤪");
            } else {
                forward = y;
                strafe  = x;
                telemetry.addData("MODE", "😴 Normal Mode");
            }

            // 3. Mecanum drive math (this makes the robot go in any direction)
            double frontLeftPower  = forward + strafe + turn;
            double frontRightPower = forward - strafe - turn;
            double backLeftPower   = forward - strafe + turn;
            double backRightPower  = forward + strafe - turn;

            // 4. Make sure no power goes over 1.0 (or under -1.0)
            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower),
                    Math.abs(backRightPower))));
            if (max > 1.0) {
                frontLeftPower  /= max; // frontLeftPower = frontLeftPower / max
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // 5. Send power to motors
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // 6. Show info on the driver station phone
            telemetry.addData("Left Stick", "Y=%.2f X=%.2f", y, x);
            telemetry.addData("Forward Power", "%.2f", forward);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.update();
        }
    }
}