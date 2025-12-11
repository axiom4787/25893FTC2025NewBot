/*
    How to Run It

    1. Put robot on floor
    2. Tape a line exactly 36 inches to the right of robot center
    3. Run this OpMode → Press PLAY
    4. Robot strafes right slowly
    5. When center of robot hits the tape → PRESS SQUARE (STOP)
    6. Look at phone → it says something like STRAFE_TICKS_PER_INCH = 49.1278
    7. Copy that number and use it in your strafe function instead of regular TICKS_PER_INCH!
 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IS - Calibrate Strafe Ticks Per Inch", group = "ISDrive")
public class StrafeCalibration extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // How far sideways you will physically measure (36 inches = 3 feet = super easy with a tape measure)
    private final double STRAFE_DISTANCE_INCHES = 36.0;

    @Override
    public void runOpMode() {

        // ==== CONNECT MOTORS ====
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Normal direction setup
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("STRAFE CALIBRATION", "Ready!");
        telemetry.addData("Step 1", "Put robot on flat floor");
        telemetry.addData("Step 2", "Put tape/mark exactly 36 inches to the RIGHT");
        telemetry.addData("Step 3", "Line up the CENTER of robot with start line");
        telemetry.addData("When ready", "Press PLAY → robot strafes right slowly");
        telemetry.addData("STOP ME", "Press SQUARE when robot center hits 36-inch mark!");
        telemetry.update();

        waitForStart();

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Use RUN_WITHOUT_ENCODER so we can read ticks while moving
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start strafing RIGHT slowly (positive strafe)
        frontLeft.setPower(0.4);
        frontRight.setPower(-0.4);
        backLeft.setPower(-0.4);
        backRight.setPower(0.4);

        telemetry.addData("→→→ STRAFING RIGHT →→→", "Go until center hits 36-inch mark!");
        telemetry.update();

        while (opModeIsActive()) {

            // Average the absolute values of the four motors (they move opposite directions when strafing)
            int avgTicks = Math.abs(
                    frontLeft.getCurrentPosition() +
                            frontRight.getCurrentPosition() +
                            backLeft.getCurrentPosition() +
                            backRight.getCurrentPosition()
            ) / 4;

            double liveTicksPerInch = avgTicks / STRAFE_DISTANCE_INCHES;

            telemetry.addData("→ STRAFING RIGHT →", "%.1f inches so far", avgTicks / liveTicksPerInch);
            telemetry.addData("Average ticks", avgTicks);
            telemetry.addData("LIVE STRAFE TICKS_PER_INCH", "%.4f", liveTicksPerInch);
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.addData("BL", backLeft.getCurrentPosition());
            telemetry.addData("BR", backRight.getCurrentPosition());
            telemetry.addData("STOP WHEN", "Center of robot = 36-inch tape → PRESS SQUARE!");
            telemetry.update();
        }

        // You pressed STOP! Grab the final number
        int finalTicks = Math.abs(
                frontLeft.getCurrentPosition() +
                        frontRight.getCurrentPosition() +
                        backLeft.getCurrentPosition() +
                        backRight.getCurrentPosition()
        ) / 4;

        double strafeTicksPerInch = finalTicks / STRAFE_DISTANCE_INCHES;

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // FINAL RESULT — THIS IS YOUR GOLDEN NUMBER FOR STRAFING!
        telemetry.clearAll();
        telemetry.addData("STRAFE CALIBRATION DONE", "Copy this number!");
        telemetry.addData("STRAFE_TICKS_PER_INCH =", "%.5f", strafeTicksPerInch);
        telemetry.addData("", "Use this in your strafe() function!");
        telemetry.addData("Example →", "private final double STRAFE_TICKS_PER_INCH = %.5f;", strafeTicksPerInch);
        telemetry.addData("Tip", "It's usually 1–5 higher than forward ticks");
        telemetry.update();

        // Keep showing forever so you can write it down or screenshot
        while (opModeIsActive()) {
            sleep(1000);
        }
    }
}