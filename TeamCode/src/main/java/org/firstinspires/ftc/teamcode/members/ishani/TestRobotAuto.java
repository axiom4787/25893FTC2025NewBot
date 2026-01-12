package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "IS - Test Robot Class (Auto)", group = "ISDrive")
public class TestRobotAuto extends LinearOpMode {

    private Robot robot;  // ← your magic robot!

    @Override
    public void runOpMode() {

        // ———————— STEP 1: CONNECT YOUR ROBOT (this does EVERYTHING!) ————————
        robot = new Robot(hardwareMap, telemetry);

        telemetry.addData("ROBOT READY", "Press PLAY to start the path!");
        telemetry.update();

        waitForStart();

        // ———————— STEP 2: YOUR AUTONOMOUS PATH — super clean! ————————
        robot.driveStraight(30);        // forward 30 inches
        robot.strafe(12);               // strafe right 12 inches
        robot.turn(90);                 // turn left 90°
        robot.driveStraight(18);        // forward 18 inches
        robot.stopAtDistance(10);       // ← STOP exactly 10 inches from wall!
        robot.turn(-90);                // turn back right 90°

        telemetry.addData("MISSION COMPLETE", "Used drive, strafe, turn, and distance sensor!");
        telemetry.update();
    }
}