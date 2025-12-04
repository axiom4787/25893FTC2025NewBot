package org.firstinspires.ftc.teamcode.FRLib.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorW {
    private final DcMotor motor;
    private final Telemetry telemetry;
    private static final double MAX_POWER = 1.0; // this should NEVER exceed 1.0, unless using a VERY different motor

    public MotorW(LinearOpMode opMode, String name, DcMotor.Direction direction) {
        this.telemetry = opMode.telemetry;
        this.motor = opMode.hardwareMap.get(DcMotor.class, name);

        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // --- Power ---
    private double capPower(double power) {
        double abs = Math.min(Math.abs(power), MAX_POWER);
        return power < 0 ? -abs : abs;
    }

    public void setPower(double power) {
        motor.setPower(capPower(power));
    }

    // --- Encoder ---
    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition(int target) {
        setTargetPosition(target);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // --- Misc ---
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public DcMotor getRawMotor() {
        return motor;
    }
}
