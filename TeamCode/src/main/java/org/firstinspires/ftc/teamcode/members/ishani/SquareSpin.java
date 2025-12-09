package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Square + Spin", group = "Auto")
public class SquareSpin extends LinearOpMode {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Wheel & Robot Specs
    // 1. WHEEL DIAMETER (in inches)
    private static final double WHEEL_DIAMETER_INCHES = 4.09;     // 96mm or 100mm wheels
    // 2. Motor encoder ticks per revolution
    private static final double TICKS_PER_REV = 537.7;          // goBILDA Yellow Jacket
    // 3. Gear ratio (how many times motor turns for 1 wheel turn)
    private static final double GEAR_RATIO = 19.2;
    // 4. Calculate how many encoder ticks = 1 inch forward
    private static final double COUNTS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_INCHES);

    // 5. TRACK WIDTH: distance between left and right wheels (center to center)
    private static final double TRACK_WIDTH_INCHES = 13.5;       // MEASURE YOUR ROBOT!
    // 6. How many ticks to turn 1 degree
    private static final double COUNTS_PER_DEGREE = (COUNTS_PER_INCH * TRACK_WIDTH_INCHES * Math.PI) / 360.0;

    // Speeds
    private static final double DRIVE_POWER = 0.3;
    private static final double TURN_POWER = 0.2;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // === 1. HARDWARE MAP ===
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // === 2. MOTOR DIRECTIONS ===
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // === 3. RESET ENCODERS ===
        resetEncoders();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready! Press START");
        telemetry.addData("COUNTS PER INCH:", "%.2f", COUNTS_PER_INCH);
        telemetry.addData("COUNTS PER DEGREE:", "%.2f", COUNTS_PER_DEGREE);
        telemetry.addData("Track Width:", "%.1f", TRACK_WIDTH_INCHES);
        telemetry.update();
        waitForStart();
        runtime.reset();

        // === 4. DRIVE 5 FEET FORWARD FIRST ===
//        driveStraight(60, DRIVE_POWER);  // 5 ft = 60 inches

        // === 5. DRIVE A PERFECT SQUARE (60 inches each side) ===
        for (int i = 0; i < 4; i++) {
            driveStraight(60, DRIVE_POWER);
            turnRight(90, TURN_POWER);
        }

        // === 6. FINAL 360° SPIN ===
        spin360(TURN_POWER);

        // === 7. DONE! ===
        stopMotors();
        telemetry.addData("Path", "COMPLETE! 5ft + Square + 360° Spin");
        telemetry.update();
        sleep(1000);
    }

    // =============== HELPER METHODS ===============

    private void driveStraight(double inches, double power) {
        int target = (int) (inches * COUNTS_PER_INCH);

        setTargetPositions(target, target, target, target);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerAll(power);

        while (opModeIsActive() && motorsBusy() && runtime.seconds() < 30) {
            telemetry.addData("Driving", "%.1f in", inches);
            telemetry.addData("Pos", "FL:%d FR:%d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnRight(double degrees, double power) {
        int target = (int) (degrees * COUNTS_PER_DEGREE);

        setTargetPositions(target, -target, target, -target);  // Right turn
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPowerAll(power);

        while (opModeIsActive() && motorsBusy() && runtime.seconds() < 10) {
            telemetry.addData("Turning", "%.0f°", degrees);
            telemetry.update();
            idle();
        }

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void spin360(double power) {
        turnRight(360, power);  // Reuse turn logic
        telemetry.addData("Spinning", "360° Celebration!");
        telemetry.update();
    }

    private void setTargetPositions(int fl, int fr, int bl, int br) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + fl);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + fr);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + bl);
        backRight.setTargetPosition(backRight.getCurrentPosition() + br);
    }

    private void setPowerAll(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopMotors() {
        setPowerAll(0);
    }

    private boolean motorsBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() ||
                backLeft.isBusy() || backRight.isBusy();
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}