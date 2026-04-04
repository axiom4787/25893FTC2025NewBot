package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Objects;

public class Hardware {
    private static DriveMotors driveMotors;
    private static DcMotorEx shooterMotor;
    private static DcMotor intakeMotor;
    private static DcMotor indexerMotor;
    private static TurretServos turretServos;
    private static Servo hoodActuator;
    private static AnalogInput axonServoEncoder;
    private static Limelight3A limelight;
    private static RevBlinkinLedDriver lights;
    private static IMU imu;

    private static HardwareMap hwMap;

    public static void init(HardwareMap hardwareMap) {
        hwMap = hardwareMap;
    }

    private Hardware() {}

    public static DriveMotors getDriveMotors() {
        if (driveMotors != null) return driveMotors;

        return driveMotors = new DriveMotors(
            hwMap.get(DcMotor.class, "front_left_drive"),
            hwMap.get(DcMotor.class, "front_right_drive"),
            hwMap.get(DcMotor.class, "back_left_drive"),
            hwMap.get(DcMotor.class, "back_right_drive")
        );
    }

    public static DcMotorEx getShooterMotor() {
        if (shooterMotor != null) return shooterMotor;

        shooterMotor = (DcMotorEx) hwMap.get(DcMotor.class, "shooter");
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setVelocityPIDFCoefficients(800, 0, 0, 200);
        return shooterMotor;
    }

    public static DcMotor getIntakeMotor() {
        if (intakeMotor != null) return intakeMotor;

        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        return intakeMotor;
    }

    public static DcMotor getIndexerMotor() {
        if (indexerMotor != null) return indexerMotor;

        indexerMotor = hwMap.get(DcMotor.class, "indexer");
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);

        return indexerMotor;
    }

    public static TurretServos getTurretServos() {
        if (turretServos != null) return turretServos;

        return turretServos = new TurretServos(
                hwMap.get(CRServo.class, "left_turret"),
                hwMap.get(CRServo.class, "right_turret")
        );
    }

    public static Servo getHoodActuator() {
        if (hoodActuator != null) return hoodActuator;

        return hoodActuator = hwMap.get(Servo.class, "linearActuator");
    }

    public static AnalogInput getAxonServoEncoder() {
        if (axonServoEncoder != null) return axonServoEncoder;

        return axonServoEncoder = hwMap.get(AnalogInput.class, "axon_servo_input");
    }

    public static Limelight3A getLimelight() {
        if (limelight != null) return limelight;

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.start();
        return limelight;
    }

    public static RevBlinkinLedDriver getLights() {
        if (lights != null) return lights;

        return lights = hwMap.get(RevBlinkinLedDriver.class, "lights");
    }

    public static IMU getIMU() {
        if (imu != null) return imu;

        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        return imu;
    }

    public static final class DriveMotors {
        public final DcMotor frontLeft;
        public final DcMotor frontRight;
        public final DcMotor backLeft;
        public final DcMotor backRight;

        public DriveMotors(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public static final class TurretServos {
        public final CRServo left;
        public final CRServo right;

        public TurretServos(CRServo left, CRServo right) {
            this.left = left;
            this.right = right;

            left.setDirection(CRServo.Direction.REVERSE);
            right.setDirection(CRServo.Direction.REVERSE);
        }
    }
}