package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IS - Strafe Left & Right", group = "ISDrive")
public class StrafeWithEncoders extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // CHANGE THIS NUMBER for your robot!
    private final double TICKS_PER_INCH = 45;

    @Override
    public void runOpMode() {

        // ==== 1. CONNECT MOTORS ====
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Reverse left side so forward is forward for everyone
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Robot Initialized!");
        telemetry.addData("TICKS_PER_INCH", TICKS_PER_INCH);
        telemetry.addData("", "Press PLAY when ready");
        telemetry.update();

        waitForStart();

        telemetry.addData("GO!", "Starting autonomous...");
        telemetry.update();
        sleep(500);

        // ==== 2. STRAFE RIGHT 18 inches ====
        strafe(18, 0.6);     // positive inches = right
        telemetry.addData("Finished", "18 inches RIGHT");
        telemetry.update();
        sleep(1000);

        // ==== 3. STRAFE LEFT 18 inches (back to start) ====
        strafe(-18, 0.2);    // negative inches = left
        telemetry.addData("Finished", "18 inches LEFT");
        telemetry.update();
        sleep(1000);

        telemetry.addData("ALL DONE", "Robot is a strafing champion!");
        telemetry.update();
    }

    // ★★★★★ STRAFE FUNCTION WITH TONS OF TELEMETRY ★★★★★
    public void strafe(double inches, double speed) {

        int targetTicks = (int) (inches * TICKS_PER_INCH);

        telemetry.addData("STRAFE", inches > 0 ? "RIGHT" : "LEFT");
        telemetry.addData("Distance", "%.1f inches", Math.abs(inches));
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.update();

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Mecanum strafe directions (RIGHT = positive)
        frontLeft.setTargetPosition(+targetTicks);
        frontRight.setTargetPosition(-targetTicks);
        backLeft.setTargetPosition(-targetTicks);
        backRight.setTargetPosition(+targetTicks);

        // Turn on run-to-position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // GO!
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        // Live updates while moving
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy() || backRight.isBusy())) {

            int fl = frontLeft.getCurrentPosition();
            int fr = frontRight.getCurrentPosition();
            int bl = backLeft.getCurrentPosition();
            int br = backRight.getCurrentPosition();

            telemetry.addData("Moving", inches > 0 ? "→ RIGHT →" : "← LEFT ←");
            telemetry.addData("Target", targetTicks);
            telemetry.addData("FL", "%d  (target %d)", fl, frontLeft.getTargetPosition());
            telemetry.addData("FR", "%d  (target %d)", fr, frontRight.getTargetPosition());
            telemetry.addData("BL", "%d  (target %d)", bl, backLeft.getTargetPosition());
            telemetry.addData("BR", "%d  (target %d)", br, backRight.getTargetPosition());
            telemetry.addData("Distance traveled", "%.1f inches", Math.abs(fl) / TICKS_PER_INCH);
            telemetry.update();
        }

        // STOP
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Switch back to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("STRAFE DONE", "%.1f inches completed!", Math.abs(inches));
        telemetry.update();
        sleep(300);
    }
}