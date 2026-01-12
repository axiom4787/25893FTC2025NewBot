package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Explore extends LinearOpMode {
    Logger logger = new Logger(Logger.LoggerMode.CRITICAL, telemetry);
    double rightDistance;
    double leftDistance;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);
        waitForStart();

        double power = 0.2;
        double freeRotateSpaceInches = 15;

        mainLoop:
        while (opModeIsActive()) {
            // forward until wall
            while (robot.distanceSensor.getDistanceInches() > freeRotateSpaceInches) {
                robot.drive.driveVector(1, 0, 0, power);
            }

            robot.drive.stop();

            // check how far right wall is
            robot.drive.turnDegreesPID(90, power, 0.1);
            rightDistance = robot.distanceSensor.getDistanceInches();

            // check how far left wall is
            robot.drive.turnDegreesPID(180, power, 0.1);
            leftDistance = robot.distanceSensor.getDistanceInches();

            // go back to "forward"
            robot.drive.turnDegreesPID(90, power, 0.1);

            // go all the way right
            robot.drive.driveDistance(MecanumDrive.Direction.RIGHT, rightDistance - freeRotateSpaceInches, power, rightDistance, false);
            // constantly check for opening
            while (robot.drive.isAnyMotorBusy()) {
                if (robot.distanceSensor.getDistanceInches() > freeRotateSpaceInches) {
                    // restart whole main loop
                    continue mainLoop;
                }
            }

            // we know wall on right side if at this point, so go back left
            robot.drive.driveDistance(MecanumDrive.Direction.LEFT, leftDistance + rightDistance - freeRotateSpaceInches, power, leftDistance + rightDistance, true);
            // constantly check for opening
            while (robot.drive.isAnyMotorBusy()) {
                if (robot.distanceSensor.getDistanceInches() > freeRotateSpaceInches) {
                    // restart whole main loop
                    continue mainLoop;
                }
            }
            // atp, we know there's no way around on either side. just turn left and repeat.
            robot.drive.turnDegreesPID(-90, power, 0.1);
        }
    }
}
