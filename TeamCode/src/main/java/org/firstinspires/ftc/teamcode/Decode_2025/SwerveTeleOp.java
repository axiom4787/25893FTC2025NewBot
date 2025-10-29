// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode_2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
public class SwerveTeleOp extends LinearOpMode {

  // Create a RobotHardware object to be used to access robot hardware.
  // Prefix any hardware functions with "robot." to access this class.
  Swerve_Drive robot = new Swerve_Drive(this); // has position navigation
  Odometry_Sensor odo = new Odometry_Sensor();
  Husky_Sensor husk = new Husky_Sensor(this); // used to navigate pickup balls
  April_Sensor april = new April_Sensor(this); // used for shooting alignment

  @Override
  public void runOpMode() {
    double drive = 0.0;
    double turn = 0.0;
    double straf = 0.0;
    try {
      // initialize all the hardware, using the hardware class. See how clean and simple this is?
      robot.SwerveInit();
      husk.initHuskyLens();
      husk.setAlgorithm("color");

      // Send telemetry message to signify robot waiting;
      waitForStart();
      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {
        husk.setBlocks(); // get the largest object
        telemetry.addData("ID ---", husk.getObjId());
        telemetry.addData("ID Top", husk.getTop());
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate
        // it)
        // In this mode the Left stick Y moves the robot rfwd and back, the Right X stick turns left
        // and right.
        robot.frontrear(GameTurn(), GameDrive());
        odo.ppo.getPosition();
        telemetry.addData("speed", GameDrive());
        telemetry.addData("turn", GameTurn());
        odo.ppo.update();
        telemetry.addData("position X", odo.ppo.getPosX());
        telemetry.addData("position Y", odo.ppo.getPosY());

        telemetry.update();
      } // end while opMode
    } catch (Exception e) {
      telemetry.addData("SwerveTeleOp Exception caught", e);
      telemetry.update();
      RobotLog.d(String.format("SwerveTeleOp e ", e));
      sleep(8000); // allow read of exception
      requestOpModeStop();
    }
  } // end runOpMode

  private double GameDrive() {
    // In this mode the Left stick Y moves the robot rfwd and back, the X stick turns left and
    // right.
    if (-gamepad1.left_stick_y > 0) {
      return Math.pow(-gamepad1.left_stick_y, 2);
    } else {
      return -Math.pow(-gamepad1.left_stick_y, 2);
    }
  } // end game drive

  private double GameTurn() {
    // returns a servo 0-.5-1 control
    return (gamepad1.right_stick_x + 1)
        / 2; // telemetry.addData("drive encoder",robot.encCntD(drive));
  }
} //  end swerve TeleOp
