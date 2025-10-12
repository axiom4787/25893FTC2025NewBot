package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Movement;

@TeleOp(name = "AprilTagTester", group = "AA_main")
public class AprilTagTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTag aprilTag = new AprilTag(hardwareMap);
        AprilTagAimer aprilAimer = new AprilTagAimer(hardwareMap);
        Movement movement = new Movement(hardwareMap);
        GamepadEx gamePadOne = new GamepadEx(gamepad1);
        GamepadEx gamePadTwo = new GamepadEx(gamepad2);
        boolean continuousAprilTagLock = false;

        waitForStart();
        while (opModeIsActive()) {
            gamePadOne.readButtons();
            gamePadTwo.readButtons();

            double turnCorrection;
            if (continuousAprilTagLock) {
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();
                turnCorrection = aprilAimer.calculateTurnPowerToBearing(bearing);
            }
            else {
                turnCorrection = 0;

            }
            movement.teleopTick(gamePadOne.getLeftX(), gamePadOne.getLeftY(), gamePadOne.getRightX(), turnCorrection);

            telemetry.addData("Y:", "Scan obelisk apriltag");
            telemetry.addData("A:", "Continuously lock into apriltag");
            telemetry.addData("B:", "Stop continuously locking into apriltag");
            telemetry.addData("Left Bumper", "Set to blue alliance apriltag");
            telemetry.addData("Right Bumper", "Set to red alliance apriltag");
            telemetry.update();

            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.Y)) {
                aprilTag.scanObeliskTag();
                telemetry.addData("This is probably only for auto,", "as we can just memorize the 3 possible patterns for teleop");
                telemetry.addData("Obelisk apriltag ID: ", aprilTag.getObeliskId());
                telemetry.update();
            }
             
            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.A)) {
                continuousAprilTagLock = true;

                telemetry.addData("Continuously locked in on", "apriltag");
                telemetry.update();
            }
            
            if (gamePadTwo.wasJustPressed(GamepadKeys.Button.B)) {
                continuousAprilTagLock = false;
            
                telemetry.addData("Stopped continuous lock in on", "apriltag");
                telemetry.update();
            }

            if(gamePadTwo.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                aprilTag.setGoalTagID(20);
                telemetry.addData("Set to", "Blue Alliance") ;
                telemetry.update();
            }
            if(gamePadTwo.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                aprilTag.setGoalTagID(24);
                telemetry.addData("Set to", "Red Alliance");
                telemetry.update();
            }
        }
    }
}
