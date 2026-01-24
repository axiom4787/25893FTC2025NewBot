package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class ThePlantRobotOpMode extends LinearOpMode {
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public DcMotor intake, shooter;
    public CRServo turretLeft, turretRight;
    public Servo linearActuator;
    public HuskyLens huskyLens;
    public IMU imu;

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    final public void runOpMode() {
        internalInitElectronics();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        opModeInit();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            opModeLoop();

            telemetry.update();
        }
    }

    abstract void opModeInit();
    abstract void opModeLoop();

    private void internalInitElectronics() {
        Config config = new Config();
        config.init(hardwareMap);

        frontLeftDrive = config.frontLeftDrive;
        frontRightDrive = config.frontRightDrive;
        backLeftDrive = config.backLeftDrive;
        backRightDrive = config.backRightDrive;
        intake = config.intake;
        shooter = config.shooter;
        turretLeft = config.turretServoLeft;
        turretRight = config.turretServoRight;
        linearActuator = config.linearActuator;
        huskyLens = config.huskyLens;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        imu = config.imu;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
}
