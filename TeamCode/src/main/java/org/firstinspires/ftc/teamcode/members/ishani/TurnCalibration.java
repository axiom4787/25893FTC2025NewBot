/*
    Run this once and it will give you the exact number of encoder ticks needed
    for a perfect 90° turn on robot.

    How to Use It
    1. Put robot on floor facing a wall or tape (0°)
    2. Run this OpMode → Press PLAY
    3. Robot turns left slowly
    4. When it is EXACTLY 90° (perfectly sideways) → PRESS SQUARE
    5. Phone shows something like TICKS_FOR_90_DEGREES = 523.4471
    6. Copy that number and use it in your turn function!
 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IS - Calibrate Turn 90°", group = "ISDrive")
public class TurnCalibration extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // You will turn exactly 90° and stop when perfectly aligned
    private final double DEGREES_TO_TURN = 90.0;

    @Override
    public void runOpMode() {

        // ==== CONNECT MOTORS ====
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData("TURN CALIBRATION", "Ready to find perfect 90°!");
        telemetry.addData("Step 1", "Put robot on flat floor");
        telemetry.addData("Step 2", "Put a piece of tape straight ahead as a reference");
        telemetry.addData("Step 3", "Face robot exactly at the tape (0°)");
        telemetry.addData("When ready", "Press PLAY → robot turns LEFT slowly");
        telemetry.addData("STOP ME", "When robot is EXACTLY 90° (perfectly sideways) → PRESS SQUARE!");
        telemetry.update();

        waitForStart();

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Start turning LEFT slowly
        frontLeft.setPower(-0.35);
        frontRight.setPower(0.35);
        backLeft.setPower(-0.35);
        backRight.setPower(0.35);

        telemetry.addData("↺ TURNING LEFT ↺", "Watching for perfect 90°...");
        telemetry.update();

        while (opModeIsActive()) {

            // Average the absolute ticks from opposite-moving wheels
            int avgTicks = Math.abs(
                    frontLeft.getCurrentPosition() +
                            frontRight.getCurrentPosition() +
                            backLeft.getCurrentPosition() +
                            backRight.getCurrentPosition()
            ) / 4;

            double liveTicksPer90 = avgTicks;

            telemetry.addData("↺ TURNING LEFT ↺", "Stop at EXACT 90°");
            telemetry.addData("Current average ticks", avgTicks);
            telemetry.addData("LIVE TICKS FOR 90°", "%.3f", liveTicksPer90);
            telemetry.addData("Estimated angle now", "%.1f°", (avgTicks / liveTicksPer90) * 90.0);
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.addData("BL", backLeft.getCurrentPosition());
            telemetry.addData("BR", backRight.getCurrentPosition());
            telemetry.addData("STOP WHEN", "Robot is PERFECTLY 90° to starting direction → PRESS SQUARE!");
            telemetry.update();
        }

        // You pressed STOP at perfect 90°!
        int finalTicks = Math.abs(
                frontLeft.getCurrentPosition() +
                        frontRight.getCurrentPosition() +
                        backLeft.getCurrentPosition() +
                        backRight.getCurrentPosition()
        ) / 4;

        double ticksFor90Degrees = finalTicks;

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // FINAL GOLDEN NUMBERS!
        telemetry.clearAll();
        telemetry.addData("TURN CALIBRATION COMPLETE", "You nailed it!");
        telemetry.addData("TICKS_FOR_90_DEGREES =", "%.4f", ticksFor90Degrees);
        telemetry.addData("TICKS_PER_DEGREE =", "%.5f", ticksFor90Degrees / 90.0);
        telemetry.addData("", "Copy one of these into your code!");
        telemetry.addData("Example 1 →", "private final double TICKS_90 = %.4f;", ticksFor90Degrees);
        telemetry.addData("Example 2 →", "int ticks = (int)(degrees * %.5f);", ticksFor90Degrees / 90.0);
        telemetry.addData("Pro Tip", "Use this instead of math formulas — it's always perfect!");
        telemetry.update();

        // Keep showing forever so you can screenshot
        while (opModeIsActive()) {
            sleep(1000);
        }
    }
}