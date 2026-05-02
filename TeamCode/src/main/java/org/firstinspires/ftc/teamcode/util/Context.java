package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.Teleop;

@Configurable
public class Context {
    public static HardwareMap hwMap;
    public static Telemetry telemetry;
    public static Follower follower;

    public static void init(OpMode runningOpMode) {
        hwMap = runningOpMode.hardwareMap;
        telemetry = runningOpMode.telemetry;
        if (runningOpMode instanceof Teleop) {
            follower = ((Teleop) runningOpMode).follower;
        } else if (runningOpMode instanceof AutoOpMode) {
            follower = ((AutoOpMode) runningOpMode).follower;
        } else {
            throw new RuntimeException();
        }
    }

    private Context() {}

    public static DriveMotors getDriveMotors() {
        return new DriveMotors(
                hwMap,
                "frontLeftDrive",
                "frontRightDrive",
                "backLeftDrive",
                "backRightDrive"
        );
    }

    public static MotorEx getShooterMotor() {
        MotorEx shooter = new MotorEx(hwMap, "shooter", Motor.GoBILDA.BARE);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setInverted(false);
        shooter.setFeedforwardCoefficients(0, 0);
        shooter.setVeloCoefficients(0, 0, 0);
        shooter.setCachingTolerance(0.01);
        return shooter;
    }

    public static VoltageSensor getVoltageSensor() {
        return hwMap.voltageSensor.iterator().next();
    }

    public static MotorEx getIntakeMotor() {
        MotorEx intake = new MotorEx(hwMap, "intake");
        intake.setInverted(true);
        intake.setCachingTolerance(0.01);
        intake.setRunMode(Motor.RunMode.RawPower);
        return intake;
    }

    public static MotorEx getIndexerMotor() {
        MotorEx indexer = new MotorEx(hwMap, "indexer");
        indexer.setInverted(false);
        indexer.setCachingTolerance(0.01);
        indexer.setRunMode(Motor.RunMode.RawPower);
        return indexer;
    }

    public static ServoEx getHoodActuator() {
        ServoEx actuator = new ServoEx(hwMap, "linearActuator");
        actuator.setCachingTolerance(0.01);
        return actuator;
    }

    public static CachedRTPAxon getTurretServo() {
        CRServoEx turretServo = new CRServoEx(hwMap, "turretServoLeft");
        turretServo.setRunMode(CRServoEx.RunMode.RawPower);
        turretServo.setInverted(true);
        turretServo.setCachingTolerance(0.02);
        AnalogInput encoder = hwMap.get(AnalogInput.class, "verySmartServo");
        CachedRTPAxon axon = new CachedRTPAxon(turretServo, encoder);
        axon.setPIDFCoefficients(0.01, 0, 0, 0.03);
        axon.forceResetTotalRotation();
        return axon;
    }

    public static Limelight3A getLimelight() {
        Limelight3A limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.start();
        return limelight;
    }

    public static RevBlinkinLedDriver getLights() {
        return hwMap.get(RevBlinkinLedDriver.class, "lights");
    }

    public static IMU getIMU() {
        IMU imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        return imu;
    }
}
