package org.firstinspires.ftc.teamcode.Decode_2025;// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

/*
 *-* Control configuration
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class _SwerveAutomaton extends LinearOpMode {

  // Create a RobotHardware object to be used to access robot hardware.
  // Prefix any hardware functions with "robot." to access this class.
  DC_Swerve_Drive robot = new DC_Swerve_Drive(this);
  DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  DC_Husky_Sensor husk = new DC_Husky_Sensor(this);


  @Override
  public void runOpMode() {

    // initialize all the hardware, using the hardware class. See how clean and simple this is?
    robot.SwerveInit();
    husk.initHuskyLens();
    husk.setAlgorithm("color");

    double topDrive = 0.0;
    double leftDrive = 0.0;
    // Send telemetry message to signify robot waiting;
    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {
      // robot.gamePadTeleOp();
      husk.setBlocks(); // get the largest object

      telemetry.addData("Id", husk.getObjId());
      topDrive = husk.topPos();
      leftDrive = husk.leftPos(); // 0 - 1
      telemetry.addLine(" ball location compensation");
      telemetry.addLine(" **************************");
      telemetry.addData("Fwd +, Bck :", topDrive);
      telemetry.addData("left,right :", leftDrive);
      telemetry.addLine("     April tag");
      telemetry.addLine(" *****************");

      }
      telemetry.addLine("  field location");
      telemetry.addLine(" *****************");

      odo.ppo.getPose();
      odo.ppo.update();
      telemetry.addData("position X", odo.ppo.getPosX());
      telemetry.addData("position Y", odo.ppo.getPosY());

      telemetry.update();
    } // end while run loop

    // sleep(500);

} // end class Swerve Automation
