package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;

/**
 * Autonomous opmode to test and adjust PID values for the robot drive.
 * Uses gamepad1 buttons/triggers to set P, I, and D terms.
 */
@Autonomous
public class PIDTest extends LinearOpMode {

    Robot robot;
    Logger logger = new Logger(Logger.LoggerMode.CRITICAL, telemetry);

    // Currently selected PID term: 'P', 'I', 'D', or '\0' for none
    char currentlyChanging = '\0';

    // State for detecting button/trigger edges
    float lastTrigger = 0f;
    boolean lastX = false, lastY = false, lastB = false, lastRB = false;

    @Override
    public void runOpMode() {
        robot = new Robot(this, logger, MecanumDrive.RobotName.KEVIN);
        logger.log(Logger.LoggerMode.STATUS, "Init!");
        logger.update();
        waitForStart();
        logger.logData(Logger.LoggerMode.STATUS, "Status", "running!");
        logger.update();


        // Start a PID turn in a separate thread (turn to 90 degrees)
        new Thread(() -> robot.drive.turnDegreesPID(robot.imu, 90, 0.2, -1)).start();

        while (opModeIsActive()) {
            handlePIDSelection();
            handleTriggerSet();
            handleReset();

            updateTelemetry();
            logger.update();

            // Save last states for edge detection
            lastTrigger = gamepad1.left_trigger;
            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastB = gamepad1.b;
            lastRB = gamepad1.right_bumper;

            sleep(50);
        }
    }

    //================================================================================
    // PID Handling
    //================================================================================

    /** Select which PID term to change based on gamepad buttons */
    private void handlePIDSelection() {
        if (gamepad1.x && !lastX) currentlyChanging = 'P';
        else if (gamepad1.y && !lastY) currentlyChanging = 'I';
        else if (gamepad1.b && !lastB) currentlyChanging = 'D';
    }

    /** Detect trigger release and update currently selected PID term */
    private void handleTriggerSet() {
        float trigger = gamepad1.left_trigger;
        if (lastTrigger > 0.01 && trigger <= 0.01 && currentlyChanging != '\0') {
            setPIDTerm(currentlyChanging, lastTrigger);
        }
    }

    /** Reset selected PID term to 0 using right bumper */
    private void handleReset() {
        if (gamepad1.right_bumper && !lastRB && currentlyChanging != '\0') {
            setPIDTerm(currentlyChanging, 0);
        }
    }

    /** Update telemetry for current state */
    private void updateTelemetry() {
        logger.logData(Logger.LoggerMode.CRITICAL, "Currently Changing", currentlyChanging == '\0' ? "None" : currentlyChanging);
        logger.logData(Logger.LoggerMode.CRITICAL, "Last Trigger Value", lastTrigger);
    }

    /** Set the PID term on the robot drive */
    private void setPIDTerm(char term, double value) {
        switch (term) {
            case 'P':
                robot.drive.turnPid.setKp(value);
                break;
            case 'I':
                robot.drive.turnPid.setKi(value);
                break;
            case 'D':
                robot.drive.turnPid.setKd(value);
                break;
        }

        logger.logData(Logger.LoggerMode.CRITICAL, "Set " + term + " to", value);

        // Small pause for feedback
        sleep(500);
    }
}
