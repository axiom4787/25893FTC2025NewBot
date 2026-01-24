package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Config {
    DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    DcMotor intake, shooter;
    CRServo turretServoLeft, turretServoRight;
    Servo linearActuator;
    HuskyLens huskyLens;
    IMU imu;
    public void init(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        turretServoLeft = hardwareMap.get(CRServo.class, "turretServoLeft");
        turretServoRight = hardwareMap.get(CRServo.class, "turretServoRight");
        turretServoLeft.setDirection(CRServo.Direction.REVERSE);
        turretServoRight.setDirection(CRServo.Direction.REVERSE);

        linearActuator = hardwareMap.get(Servo.class, "linearActuator");
        linearActuator.setDirection(Servo.Direction.FORWARD);

        huskyLens = hardwareMap.get(HuskyLens.class, "ebk"); // elite ball knowledge

        imu = hardwareMap.get(IMU.class, "imu");
    }

    public static double calculateTurret(HuskyLens.Block target) {
        double base = -((target.x / 160f) - 1f) * 0.5f;
        return Math.signum(base) * Math.pow(Math.abs(base), 1.4f) * 2.5f;
    }

    public static double calculateHood(HuskyLens.Block target) {
        double base = (target.y - 110f) / 160f * 0.035f;
        return Math.signum(base) * Math.pow(Math.abs(base), 1.4f) * 12f;
    }

    public static HuskyLens.Block getTargetBlock (HuskyLens huskyLens) {
        HuskyLens.Block target = null;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        double size = 0f;
        for (int i = 0; i < blocks.length; i++) {
            HuskyLens.Block block = blocks[i];
            if (block.height * block.width > size) {
                target = block;
                size = block.height * block.width;
            }
        }

        return target;
    }
}