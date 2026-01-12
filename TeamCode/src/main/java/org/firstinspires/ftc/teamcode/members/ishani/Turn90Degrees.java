package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IS - Turn 90° Left & Right", group = "ISDrive")
public class Turn90Degrees extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // YOU MUST FILL THESE IN AFTER YOU RUN THE CALIBRATOR!
    private final double TICKS_PER_INCH = 47.8321;        // ← Your number from the calibrator!
    private final double WHEEL_BASE_INCHES = 13.5;        // Distance between left & right wheels (measure yours!)

    // This gets calculated automatically from the two numbers above
    private double TICKS_PER_90_DEGREES;

    @Override
    public void runOpMode() {

        // Connect motors
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Calculate how many ticks = 90° turn
        TICKS_PER_90_DEGREES = (Math.PI * WHEEL_BASE_INCHES / 4) * TICKS_PER_INCH;
        // Math explanation: full 360° = π × wheel base distance → 90° = 1/4 of that

        telemetry.addData("Robot Ready!", "Turn 90° Test");
        telemetry.addData("TICKS_PER_INCH", "%.4f", TICKS_PER_INCH);
        telemetry.addData("Wheel base", "%.2f inches", WHEEL_BASE_INCHES);
        telemetry.addData("Ticks needed for 90°", "%.1f", TICKS_PER_90_DEGREES);
        telemetry.addData("", "Press PLAY to start turning!");
        telemetry.update();

        waitForStart();

        // TURN LEFT 90°
        turnDegrees(90, 0.5);
        sleep(1000);

        // TURN RIGHT 180° (back more than 90 so you see it clearly)
        turnDegrees(-180, 0.5);
        sleep(1000);

        // TURN LEFT 90° again → back to original direction
        turnDegrees(90, 0.5);

        telemetry.addData("ALL DONE", "Perfect square completed!");
        telemetry.update();
    }

    // ★★★★★ SUPER TELEMETRY TURN FUNCTION ★★★★★
    public void turnDegrees(double degrees, double speed) {

        boolean turningLeft = degrees > 0;
        int targetTicks = (int) (Math.abs(degrees) / 90.0 * TICKS_PER_90_DEGREES);

        telemetry.addData("TURN", turningLeft ? "LEFT ↺" : "RIGHT ↻");
        telemetry.addData("Degrees", "%.1f°", Math.abs(degrees));
        telemetry.addData("Target ticks", "%d", targetTicks);
        telemetry.update();
        sleep(500);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Left turn  → left motors backward, right motors forward
        // Right turn → opposite
        int direction = turningLeft ? -1 : 1;

        frontLeft.setTargetPosition(direction * -targetTicks);
        frontRight.setTargetPosition(direction * targetTicks);
        backLeft.setTargetPosition(direction * -targetTicks);
        backRight.setTargetPosition(direction * targetTicks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy())) {

            int fl = frontLeft.getCurrentPosition();
            int fr = frontRight.getCurrentPosition();
            int avg = Math.abs(fl + fr) / 2;

            telemetry.addData("Turning", turningLeft ? "↺ LEFT" : "↻ RIGHT");
            telemetry.addData("Target", "%d ticks (%.0f°)", targetTicks, Math.abs(degrees));
            telemetry.addData("Progress", "%.1f°", (avg * 90.0 / TICKS_PER_90_DEGREES));
            telemetry.addData("FL", "%d", fl);
            telemetry.addData("FR", "%d", fr);
            telemetry.addData("BL", "%d", backLeft.getCurrentPosition());
            telemetry.addData("BR", "%d", backRight.getCurrentPosition());
            telemetry.update();
        }

        // STOP!
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("TURN FINISHED", "%.1f° completed perfectly!", Math.abs(degrees));
        telemetry.update();
        sleep(400);
    }
}