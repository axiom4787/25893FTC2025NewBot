package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;

@Autonomous
public class LogTest extends LinearOpMode {
    Logger logger = new Logger(Logger.LoggerMode.DETAILED, telemetry);
    @Override
    public void runOpMode() {
        logger.logData(Logger.LoggerMode.STATUS, "Status", "init");
        logger.update();
        sleep(1000);
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);
        waitForStart();
        logger.logData(Logger.LoggerMode.STATUS, "Status", "active");
        logger.update();
        sleep(1000);

        if (opModeIsActive()) {
            robot.drive.turnDegreesPID(robot.imu, 90, 1, 1);
            logger.update();
            logger.log(Logger.LoggerMode.STATUS, "PID done");
            logger.update();
            sleep(1000);
        }

        logger.logData(Logger.LoggerMode.STATUS, "Status", "done");
        logger.update();
        sleep(1000);
//        logger.log(Logger.LoggerMode.DETAILED, "hey!");
//        sleep(3000);
//        logger.log(Logger.LoggerMode.DETAILED, "hi!");
//        sleep(3000);
//        logger.logData(Logger.LoggerMode.DETAILED, "status", "friend!");
//        sleep(3000);
//        logger.logData(Logger.LoggerMode.DETAILED, "status", "confirmed that btw update.");
//        sleep(3000);
//        logger.update();
//
//        // gonna do some work now, eg moving a motor
//
//        logger.logData(Logger.LoggerMode.DETAILED, "status", "post update");
//
//        logger.update();
//
//        logger.log(Logger.LoggerMode.DETAILED, "ITS A NEW WORLD!!!!");
//
//        logger.update();
//        // that work also did logging, and updated internally, like maybe it was a loop
//
//        sleep(30000);
    }
}
