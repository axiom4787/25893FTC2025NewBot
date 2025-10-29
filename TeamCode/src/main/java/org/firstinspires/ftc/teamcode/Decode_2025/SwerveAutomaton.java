// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode_2025;

/*
 *-* Control configuration
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class SwerveAutomaton extends LinearOpMode {

  // Create a RobotHardware object to be used to access robot hardware.
  // Prefix any hardware functions with "robot." to access this class.
  Swerve_Drive robot = new Swerve_Drive(this);
  Odometry_Sensor odo = new Odometry_Sensor();
  Husky_Sensor husk = new Husky_Sensor(this);
  April_Sensor april = new April_Sensor(this);

  @Override
  public void runOpMode() {

    // initialize all the hardware, using the hardware class. See how clean and simple this is?
    robot.SwerveInit();
    husk.initHuskyLens();
    husk.setAlgorithm("color");
    april.initAprilTag();
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
      if (april.aprilRecognize()) {
        telemetry.addData("Tag", april.getTagName());
        telemetry.addData("Range", april.getTagRange());
        telemetry.addData("Bearing", april.getTagBearing());
      }
      telemetry.addLine("  field location");
      telemetry.addLine(" *****************");

      odo.ppo.getPosition();
      odo.ppo.update();
      telemetry.addData("position X",odo.ppo.getPosX());
      telemetry.addData("position Y", odo.ppo.getPosY());

      telemetry.update();
    } // end while run loop

    // sleep(500);
    telemetry.addLine("Auto Complete");
    telemetry.update();
  } // end run op mode
} // end class Swerve Automation
