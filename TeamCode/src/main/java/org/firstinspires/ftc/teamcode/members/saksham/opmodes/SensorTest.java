package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Logger.LoggerMode.CRITICAL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;

@Autonomous
public class SensorTest extends LinearOpMode {
    Logger logger = new Logger(CRITICAL, telemetry);

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);
        waitForStart();

        while (opModeIsActive()) {
            logger.logData(CRITICAL, "btn", robot.touchSensor.isPressed());
            logger.logData(CRITICAL, "distance", robot.distanceSensor.getDistanceInches());
            logger.update();
        }
    }
}
