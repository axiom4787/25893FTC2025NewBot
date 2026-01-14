package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.VelocityProfiler4Stream;

@TeleOp
public class MecanumLauncher extends OurOpmode{
    Launcher launcher;
    MecanumDrive drive;
    IMUW imu;
    double power = 0.2;
    VelocityProfiler4Stream profiler = new VelocityProfiler4Stream(0.075);

    @Override
    protected void Loop() {
        drive.driveVectorField(
                profiler.velocityProfileInput1(-gamepad1.left_stick_y),
                profiler.velocityProfileInput2(gamepad1.left_stick_x),
                profiler.velocityProfileInput3(gamepad1.right_stick_x),
                power,
                imu);


        launcher.launchTeleOp(gamepad1.rightBumperWasPressed());
        
        if (gamepad1.bWasPressed()){
            launcher.stopFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Stopped");
        }

        if (gamepad1.leftBumperWasPressed()){
            imu.resetYaw();
            logger.logData("imu","reset");
        }
      if(gamepad1.xWasPressed()){
          power = power + 0.1;
        } else if (gamepad1.aWasPressed()) {
            power = power -0.1;
        }
        if (gamepad1.yWasPressed()){
            launcher.spinUpFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Spinning Up");
        }

        logVariables();
    }



    private void logVariables() {
        logger.logData(Logger.LoggerMode.STATUS,"FlyWheelVelocity",launcher.getLauncher().getVelocity());
        logger.logData("TargetVelocity",launcher.getLauncherTargetVelocity());
        logger.logData(Logger.LoggerMode.STATUS,"Launch-state",launcher.getLaunchStatesT());
        logger.logData("FrontLeft",drive.frontLeft.getRawMotor().getPower());
        logger.logData("FrontRight",drive.frontRight.getRawMotor().getPower());
        logger.logData("RearLeft",drive.rearLeft.getRawMotor().getPower());
        logger.logData("RearRight",drive.rearRight.getRawMotor().getPower());
        logger.update();

    }

    @Override
    protected void initialize() {
        logger = new Logger(telemetry);
        launcher = new Launcher(this);
        imu = new IMUW(hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        drive = new MecanumDrive(this,logger, MecanumDrive.RobotName.BOB,imu);

        drive.setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.resetYaw();
        logger.logData(Logger.LoggerMode.CRITICAL,"Status","Init");
        logger.update();
    }
}
