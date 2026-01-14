package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.VelocityProfilerDynamic;

@TeleOp
public class MecanumLauncherV2 extends OurOpmode{
    // init variables
    Launcher launcher;
    MecanumDrive drive;
    IMUW imu;
    double power;
    VelocityProfilerDynamic profiler = new VelocityProfilerDynamic(0.075);
    // made 2 different possibilities for what the driver wants, Field Relative Or Whithout Field Relative, which is normal.
    enum DriverPreference {
        FIELD_RELATIVE,
        NORMAL
    }
    double fastPower = 0.82;
    double normalPower = 0.5;

    DriverPreference preference = DriverPreference.NORMAL;
    @Override
    protected void Loop() {
        // this if the rightBumper is constantly pressed, then do the power o.82, otherwise, 0.5
        power = gamepad1.left_bumper ? fastPower : normalPower;
        // if the leftBumper was pressed, it checks what state the driver preference is and converts it to the other one
        if (gamepad2.leftBumperWasPressed()) {
            preference = (preference == DriverPreference.NORMAL)
                    ? DriverPreference.FIELD_RELATIVE
                    : DriverPreference.NORMAL;
        }


        driveBasedOnPreferences();


        launcher.launchTeleOp(gamepad1.rightBumperWasPressed());
        
        if (gamepad1.bWasPressed()){
            launcher.stopFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Stopped");
        }

        if (gamepad2.yWasPressed()){
            imu.resetYaw();
        }

        if (gamepad1.yWasPressed()){
            launcher.spinUpFlywheel();
            logger.logData(Logger.LoggerMode.CRITICAL,"Flywheel","Spinning Up");
        }

        logVariables();
    }

    private void driveBasedOnPreferences() {
        // this says if the preference is field relative, the drive field relative, if the preference is normal, drive normal
        switch  (preference) {
            case FIELD_RELATIVE:  drive.driveVectorField(
                    profiler.velocityProfile("Left Stick Y", -gamepad1.left_stick_y),
                    profiler.velocityProfile("Left Stick X", gamepad1.left_stick_x),
                    profiler.velocityProfile("Right Stick X", -gamepad1.right_stick_x),
                    power, imu
            ); break;


            case NORMAL : drive.driveVector(
                    profiler.velocityProfile("Left Stick Y", -gamepad1.left_stick_y),
                    profiler.velocityProfile("Left Stick X", gamepad1.left_stick_x),
                    profiler.velocityProfile("Right Stick X", -gamepad1.right_stick_x),
                    power); break;
        }
    }


    private void logVariables() {
        // logs all important information the driver needs
        logger.logData(Logger.LoggerMode.STATUS,"FlyWheelVelocity",launcher.getLauncher().getVelocity());
        logger.logData("Driver Preference",preference);
        logger.logData("IMU Yaw", imu.getYaw());
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
        // profiler.addInput means when you call the velocityprofiling method, these are the appropriate names for the inputs that the profiler will recognise
        profiler.addInput("Left Stick Y");
        profiler.addInput("Left Stick X");
        profiler.addInput("Right Stick X");
        //initializing everything else
        logger = new Logger(telemetry);
        launcher = new Launcher(this);
        drive = new MecanumDrive(this,logger, MecanumDrive.RobotName.BOB,imu);
        imu = new IMUW(hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        drive.setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        imu.resetYaw();
        logger.logData(Logger.LoggerMode.CRITICAL,"Status","Init");
        logger.update();
    }
}
