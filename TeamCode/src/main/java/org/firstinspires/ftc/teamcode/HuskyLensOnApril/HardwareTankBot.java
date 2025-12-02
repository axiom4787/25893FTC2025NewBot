// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.HuskyLensOnApril;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Hardware class for a simple tank drive robot with a HuskyLens. */
public class HardwareTankBot {
  // Motor Definitions

  public DcMotor leftDrive = null;
  public DcMotor rightDrive = null;
  public HuskyLens huskylens = null;

  // Hardware map object
  HardwareMap hwMap = null;

  /** Initialize standard Hardware interfaces */
  public void init(HardwareMap ahwMap) {
    hwMap = ahwMap;

    // Define and Initialize Motors
    leftDrive = hwMap.get(DcMotor.class, "left_drive"); // Name needs to match config
    rightDrive = hwMap.get(DcMotor.class, "right_drive"); // Name needs to match config
    huskylens = hwMap.get(HuskyLens.class, "huskylens"); // Name needs to match config

    // Set Motor Directions
    // Assuming motors are mounted to move forward when set to the directions below.
    leftDrive.setDirection(DcMotor.Direction.FORWARD);
    rightDrive.setDirection(DcMotor.Direction.REVERSE);

    // Set all motors to zero power
    leftDrive.setPower(0);
    rightDrive.setPower(0);

    // Set motors to run without encoders (unless you are using encoders for odometry/distance)
    leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // Ensure HuskyLens is operational and configured
    if (!huskylens.knock()) {
      // Handle communication failure in the OpMode
      throw new RuntimeException("HuskyLens communication failed!");
    }
    huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
  }

  /**
   * Tank Drive Method: Sets motor powers based on desired forward speed and turn speed.
   *
   * @param forwardSpeed The desired speed to move forward/backward (from -1 to 1).
   * @param turnSpeed The desired speed to turn left/right (from -1 to 1).
   */
  public void drive(double forwardSpeed, double turnSpeed) {
    double leftPower = forwardSpeed + turnSpeed;
    double rightPower = forwardSpeed - turnSpeed;

    if (Math.abs(leftPower) > 1) leftPower = (leftPower > 0) ? 1 : -1;
    if (Math.abs(rightPower) > 1) rightPower = (rightPower > 0) ? 1 : -1;

    leftDrive.setPower(leftPower);
    rightDrive.setPower(rightPower);
  }
}
