package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.DriveConstants.DriveConstants.TICKS_PER_INCH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem {

    private final DcMotor fl2, fr3, bl0, br1;
    private double speedMultiplier = 1.0;

    double lastLfPower = 0;
    double lastLbPower = 0;
    double lastRfPower = 0;
    double lastRbPower = 0;

    public DriveSubsystem(HardwareMap hardwareMap) {
        // Initialize motors from hardware map ------------------------
        fl2  = hardwareMap.get(DcMotor.class, "lf");
        fr3 = hardwareMap.get(DcMotor.class, "rf");
        bl0   = hardwareMap.get(DcMotor.class, "lb");
        br1  = hardwareMap.get(DcMotor.class, "rb");

        // Set motor directions (adjust if movement is inverted) ----------
        fl2.setDirection(DcMotor.Direction.REVERSE);
        bl0.setDirection(DcMotor.Direction.REVERSE);
        fr3.setDirection(DcMotor.Direction.FORWARD);
        br1.setDirection(DcMotor.Direction.FORWARD);

        // Set motor behavior ----------------------------------------------
        fl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure encoders are enabled ----------------------------------------------
        fl2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double forward, double strafe, double turn) {

        strafe *= 1.1; // minor correction for imperfect strafing

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        double lfPower = (forward + strafe + turn) / denominator * speedMultiplier;
        double lbPower = (forward - strafe + turn) / denominator * speedMultiplier;
        double rfPower = (forward - strafe - turn) / denominator * speedMultiplier;
        double rbPower = (forward + strafe - turn) / denominator * speedMultiplier;

        lastLbPower = lbPower;
        lastLfPower = lfPower;
        lastRbPower = rbPower;
        lastRfPower = rfPower;

        fl2.setPower(lfPower);
        fr3.setPower(rfPower);
        bl0.setPower(lbPower);
        br1.setPower(rbPower);
    }

    public void toggleSlowMode() {
        speedMultiplier = (speedMultiplier == 1.0) ? 0.5 : 1.0;
    }

    public void stop() {
        fl2.setPower(0);
        bl0.setPower(0);
        fr3.setPower(0);
        br1.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Motors -----");
        telemetry.addData("FL Power", lastLfPower);
        telemetry.addData("FR Power", (-1 *lastRfPower));
        telemetry.addData("BL Power", lastLbPower);
        telemetry.addData("BR Power", lastRbPower);
    }

    public void periodic() {
        // Add odometry later if needed
    }

    // ------------------------ AUTONOMOUS HELPERS --------------------

    public void resetEncoders() {
        fl2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunToPositionMode() {
        fl2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setTargetForwardInches(double inches, double power) {
        int moveTicks = (int) Math.round(inches * TICKS_PER_INCH);

        fl2.setTargetPosition(fl2.getCurrentPosition() + moveTicks);
        fr3.setTargetPosition(fr3.getCurrentPosition() + moveTicks);
        bl0.setTargetPosition(bl0.getCurrentPosition() + moveTicks);
        br1.setTargetPosition(br1.getCurrentPosition() + moveTicks);

        fl2.setPower(power);
        fr3.setPower(power);
        bl0.setPower(power);
        br1.setPower(power);
    }
}
