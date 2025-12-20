package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    public Drivetrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "LFM");
        leftBack = hardwareMap.get(DcMotor.class, "LBM");
        rightFront = hardwareMap.get(DcMotor.class, "RFM");
        rightBack = hardwareMap.get(DcMotor.class, "RBM");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double rotate) {
        double lfPower = forward + strafe + rotate;
        double lbPower = forward - strafe + rotate;
        double rfPower = forward - strafe - rotate;
        double rbPower = forward + strafe - rotate;

        double maxPower = Math.max(Math.abs(lfPower), Math.max(Math.abs(lbPower),
                Math.max(Math.abs(rfPower), Math.abs(rbPower))));

        if (maxPower > 1.0) {
            lfPower /= maxPower;
            lbPower /= maxPower;
            rfPower /= maxPower;
            rbPower /= maxPower;
        }

        leftFront.setPower(lfPower);
        leftBack.setPower(lbPower);
        rightFront.setPower(rfPower);
        rightBack.setPower(rbPower);
    }

    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
