package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Config {
    public DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    public DcMotor intake, shooter;
    public CRServo turretServoLeft, turretServoRight;
    public Servo linearActuator;
    public HuskyLens huskyLens;
    public IMU imu;

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
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
}