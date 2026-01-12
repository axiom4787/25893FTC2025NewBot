package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "IS - Robot Class TeleOp", group = "ISDrive")
public class TestRobotTeleOp extends LinearOpMode {

    private Robot robot;  // ← your ultimate robot!

    @Override
    public void runOpMode() {

        // ———————— CONNECT YOUR ROBOT (this sets up motors, IMU, distance sensor!) ————————
        robot = new Robot(hardwareMap, telemetry);

        telemetry.addData("TELEOP READY", "Robot-relative driving");
        telemetry.addData("Left stick", "Move forward/back + strafe");
        telemetry.addData("Right stick X", "Turn");
        telemetry.addData("Press A", "Auto-stop 10 inches from wall!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ———————— READ GAMEPAD ————————
            double drive  = -gamepad1.left_stick_y;  // forward/back
            double strafe = gamepad1.left_stick_x;   // left/right
            double turn   = gamepad1.right_stick_x;  // rotate

            // ———————— SEND POWER TO MOTORS — CLEAN & SAFE! ————————
            // We use setDrivePower() instead of touching private motors
            robot.setDrivePower(
                    drive + strafe + turn,   // frontLeft
                    drive - strafe - turn,   // frontRight
                    drive - strafe + turn,   // backLeft
                    drive + strafe - turn    // backRight
            );

            // ———————— PRESS A = AUTO-STOP 10 INCHES FROM WALL ————————
            if (gamepad1.a) {
                robot.stopAtDistance(10);  // ← perfect every time!
            }

            // ———————— LIVE TELEMETRY ON PHONE ————————
            telemetry.addData("Distance Sensor", "%.1f inches", robot.getDistanceInches());
            telemetry.addData("Robot Heading", "%.1f°", robot.getHeading());
            telemetry.addData("Controls", "Left stick = move | Right stick = turn");
            telemetry.addData("Press A", "→ Stop 10 inches from wall");
            telemetry.update();
        }
    }
}