package org.firstinspires.ftc.teamcode.Boilerplate;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    public IMU imu;
    static Config config = new Config();
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    final public void runOpMode() {
        internalInitElectronics();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        opModeInit();

        waitForStart();
        runtime.reset();

        opModeRunOnce();
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            opModeRunLoop();

            telemetry.update();
        }
    }

    public abstract void opModeInit();
    public abstract void opModeRunOnce();
    public abstract void opModeRunLoop();

    private void internalInitElectronics() {
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

        imu = config.imu;
    }
}
