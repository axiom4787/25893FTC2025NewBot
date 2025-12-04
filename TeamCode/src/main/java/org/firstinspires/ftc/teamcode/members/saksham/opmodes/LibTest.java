package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;

@Autonomous
public class LibTest extends LinearOpMode {
    Logger logger = new Logger(Logger.LoggerMode.CRITICAL, telemetry);

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);
        MecanumDrive drive = robot.drive;
        waitForStart();

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "driveDistanceTank, 60/60 forward");
        logger.update();
        drive.driveDistanceTank(60, 60, 0.2f, 5, true);

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "driveDistanceTank, 60 backward");
        logger.update();
        drive.driveDistance(MecanumDrive.Direction.BACKWARD, 60, 0.2f, 5, true);

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "turnDegreesPID 90 counter");
        logger.update();
        drive.turnDegreesPID(robot.imu, -90, 0.2f, 0.01f);

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "strafe right 60 inch");
        logger.update();
        drive.driveDistance(MecanumDrive.Direction.RIGHT, 60, 0.2f, 10, true);

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "strafe left 60 inch");
        logger.update();
        drive.driveDistance(MecanumDrive.Direction.LEFT, 60, 0.2f, 10, true);

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "turnpid 90 deg clockwise");
        logger.update();
        drive.turnDegreesPID(robot.imu, 90, 0.2f, 0.01f);

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "driveStraight 60 inches");
        logger.update();
        drive.driveStraight(robot.imu, 0, 60, 0.2f, 10);

        logger.logData(Logger.LoggerMode.CRITICAL, "CurTest", "driveStraight -60 inches (backwards)");
        logger.update();
        drive.driveStraight(robot.imu, 0, -60, 0.2f, 10);
    }
}
