package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;

@TeleOp
public class FieldCentric extends LinearOpMode {
    Logger logger = new Logger(Logger.LoggerMode.DETAILED, telemetry);
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double yaw = gamepad1.right_stick_x;

            logger.logData(Logger.LoggerMode.CRITICAL, "x", x);
            logger.logData(Logger.LoggerMode.CRITICAL, "y", y);

            robot.drive.driveVectorField(y, x, yaw, 1, robot.imu);
        }
    }
}
