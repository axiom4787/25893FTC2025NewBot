// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode_2025;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This Class uses the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, Object Colors
 *
 * HuskyLens is configured for a name of "huskylens".
 * camera image size is 320 by 240
 */

public class Husky_Sensor {

  private final int READ_PERIOD = 1;

  private HuskyLens huskyLens;

  // Default constructor
  public Husky_Sensor(LinearOpMode opmode) {
    hskOp = opmode;
  }

  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode hskOp = null;
  /*
   * you may set the rate Limit. Rate limits the reads solely to allow a user time to observe
   * what is happening on the Driver Station telemetry.
   * Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
   *  Typical applications would not likely rate limit.
   */
  /*
   * color - object color recognition
   * april - april tag recognition
   */
  private String HuskLensAlgorithm = "color";

  enum b {
    purple,
    green
  };

  private int objId = 0;
  private int height = 0;
  private int width = 0;
  private int left = 0;
  private int top = 0;
  private int x = 0;
  private int y = 0;
  private int IdSeen = 0;
  private int GTop = 0;
  private int GId = 0;
  public int green = 1;
  public int purple = 3;

  public void initHuskyLens() {
    huskyLens = hskOp.hardwareMap.get(HuskyLens.class, "huskyLens");

    /*
     * Immediately expire so that the first time through we'll do the read.
     */
    // .expire();

    /*
     * Basic check to see if the device is alive and communicating.  This is not
     * technically necessary here as the HuskyLens class does this in its
     * doInitialization() method which is called when the device is pulled out of
     * the hardware map.  However, sometimes it's unclear why a device reports as
     * failing on initialization.  In the case of this device, it's because the
     * call to knock() failed.
     */
    if (!huskyLens.knock()) {
      hskOp.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
    } else {
      hskOp.telemetry.addData(">>", "Husky Lens camera initiated");
    }

    hskOp.telemetry.update();
  }

  public void setAlgorithm(String algor) {
    if (algor == "color" || algor == "Color") HuskLensAlgorithm = "color";
    if (algor == "april" || algor == "April") HuskLensAlgorithm = "april";
    if (algor == "track" || algor == "Track") HuskLensAlgorithm = "track";
  }

  // get the blocks from camera and set variables from the largest seen object
  // setBlocks may not retrieve from a singular object as perceived by movement
  // towards object from multiple seen objects
  public void setBlocks() {
    /*
     * The device uses the concept of an algorithm to determine what types of
     * objects it will look for and/or what mode it is in.
     * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
     */
    if (HuskLensAlgorithm == "april")
      huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    if (HuskLensAlgorithm == "color")
      huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    if (HuskLensAlgorithm == "track")
      huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

    /*
     * All algorithms, except for LINE_TRACKING, return a list of Blocks where aBlock represents the outline of a recognized object along with its ID number.
     * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
     *
     *  referenced in the header comment above for more information on IDs and how to
     * assign them to objects.
     *
     * Returns an empty array if no objects are seen.
     */
    HuskyLens.Block[] blocks = huskyLens.blocks();
    GTop = 0;
    GId = 0; // reset greater than
    IdSeen = blocks.length; // return the largest size
    for (int i = 0; i < blocks.length; i++) {
      if (blocks[i].id == green || blocks[i].id == purple) {
        if (blocks[i].top > GTop) { // larger = closest ball
          GTop = blocks[i].top;
          GId = blocks[i].id;
          objId = blocks[i].id;
          height = blocks[i].height;
          width = blocks[i].width;
          left = blocks[i].left;
          top = blocks[i].top;
          x = blocks[i].x;
          y = blocks[i].y;
        } else {
          objId = 0;
        } // save largest
      } // OK purple or green set green and purple to object Id
    } // // return the largest object found
  } // end set blocks

  public int GId() {
    return GId;
  }

  public int GTop() {
    return GTop;
  }

  public int numObj() {
    return IdSeen;
  }

  public int getObjId() {
    return objId;
  }

  public int getHeightId() {
    return height;
  }

  public int getWidth() {
    return width;
  }

  public int getLeft() {
    return left;
  }

  public int getTop() {
    return top;
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }

  // movement calculations
  // Size position in image returned as area max 76800
  public int size() {
    return height * width;
  }

  // Top 0 bottom 240 noralized  0 - 1.0
  public double topPos() {
    return (double) top / 240.0;
  }

  // Left 0 right 320 return value -1 to 1 from center bottom
  public double leftPos() {
    return (double) left / 320.0;
  }

  private double map(double value) {
    /* simplified -1 + value * 2 from following:
    Output value = outputStart +
     ((value - inputStart) / (inputEnd - inputStart))
     * (outputEnd - outputStart) */
    return -1 + value * 2;
  }
} // end Husky decode class
