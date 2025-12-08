package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FRLib.hardware.Distance2mW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

@Autonomous
public class DriveUntilSeeSomethingVishruth extends LinearOpMode {

    Logger logger = new Logger(Logger.LoggerMode.CRITICAL,telemetry);
    Robot robot;
    IMUW imu;
    Distance2mW distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this,logger, MecanumDrive.RobotName.KEVIN);
        imu = new IMUW(hardwareMap,"imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        distanceSensor = new Distance2mW(hardwareMap,"distance_sensor");
        logger.logData(Logger.LoggerMode.STATUS,"Hardware","Initialized");
        logger.update();
        waitForStart();
        robot.drive.driveStraight(robot.imu,0,60,0.2f,100);
        while (opModeIsActive()){
            if (distanceSensor.getDistance(DistanceUnit.INCH) < 10){
                logger.logData(Logger.LoggerMode.CRITICAL,"Sensed Object too close. Stopping",distanceSensor.getDistanceInches() + "Inch away");
                logger.update();
                robot.drive.brake(100);
                robot.drive.stop();
            } else {
                logger.logData(Logger.LoggerMode.STATUS,"driving","Straight");
                logger.update();
                logger.logData(Logger.LoggerMode.CRITICAL,"Sensed Object is",distanceSensor.getDistanceInches() + "cm away");
                logger.update();
            }
        }

    }
}
