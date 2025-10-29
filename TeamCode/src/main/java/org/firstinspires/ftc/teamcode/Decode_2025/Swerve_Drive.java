// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode_2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Swerve_Drive {
  private LinearOpMode myOp = null;

  // Define a constructor that allows the OpMode to pass a reference to itself.
  Swerve_Drive(LinearOpMode opmode) {
    myOp = opmode;
  }

  private Odometry_Sensor odo = new Odometry_Sensor();

  // *** - *** -
  // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
  public DcMotorEx lfDrive = null;
  public DcMotorEx rfDrive = null;
  // ---
  public Servo lfTurn = null;
  public Servo rfTurn = null;
  // ---
  public AnalogInput lfServo = null;
  // public AnalogInput lrServo = null;
  public AnalogInput rfServo = null;
  // public AnalogInput rrServo = null;
  // ---
  // Swerve chassis constants
  private static final double wDia = 96.0; // mm
  private static final double wCir = wDia * 3.141592; // 301.593 wheel circumference
  public static final double mEnc = 537.7; // PPR
  private static final double mRPM = 312; // RPM 5.2 rps
  private static final double cRadius = 203.2; // chassis radius mm from center for turning
  private static final double gearRatio = 1.7; // gear ratio
  private static final double wheelBaseWidth = 298.45;
  private static final double trackWidth = 323.85;
  // outside wheel angle from inside turn aout=ain*ratio
  private final double ackermanRatio = getAckermanRatio(wheelBaseWidth, trackWidth);
  private static final double servoDegVolt = .0045;
  private static final double SA180 = 1.625;
  public double lfsa, rfsa; // global servo potentiometer voltages
  public boolean cutSpeed = false;
  // ---
  // KPI PID controller variables
  private static final double Kp = 0.5; // proportional K gain
  private static final double ki = 0.1; // integral K gain
  private final double kd = 0.2; // derivative k gain
  protected boolean anglewrap = false; //
  // pid controllers set set with values above
  PIDController lfController = new PIDController(Kp, ki, kd);
  PIDController rfController = new PIDController(Kp, ki, kd);
  // ---
  private double stfAng = 0.0; // Saved straf angle
  protected boolean stfDrv = true; // true for drive false for straf
  // ---
  // Global variables -

  // Methods used in Swerve  robot
  // -----------------------------
  public void SwerveInit() {
    odo.DoInit();
    // ---
    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();
    // Define and Initialize Motors (note: need to use reference to actual OpMode).
    lfDrive = myOp.hardwareMap.get(DcMotorEx.class, "LFM");
    rfDrive = myOp.hardwareMap.get(DcMotorEx.class, "RFM");

    // To drive forward, most robots need the motor on one side to be reversed, because the axles
    // point in opposite directions.
    // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on
    // your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90
    // Deg drives may require direction flips
    lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
    rfDrive.setDirection(DcMotorEx.Direction.REVERSE);

    // set encoders to zero a stop is performed so set either WITHOUT or WITH encoders
    // if not the motors are stopped and will not start !!!
    lfDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    lfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    rfDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    rfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    // use braking to slow the motor down faster
    lfDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    rfDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    // Define and initialize ALL installed servos.
    lfTurn = myOp.hardwareMap.get(Servo.class, "LFS");
    rfTurn = myOp.hardwareMap.get(Servo.class, "RFS");
    // limit the turn range of servos
    lfTurn.scaleRange(0.3, 0.67); // set limits of servo
    rfTurn.scaleRange(0.3, 0.67); // set limits of servo

    lfServo = myOp.hardwareMap.get(AnalogInput.class, "LFP");
    rfServo = myOp.hardwareMap.get(AnalogInput.class, "RFP");
    myOp.telemetry.addLine("Ports Initialized");
    myOp.telemetry.update();
    }

  // set robot to turn around center
  public void posRot() { // position to rotate robot - analog fb avail
    ElapsedTime timer = new ElapsedTime();
    lfTurn.setPosition(0.43); // offset calibration
    rfTurn.setPosition(0.57); // wheel at -45 deg

    lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
    rfDrive.setDirection(DcMotorEx.Direction.FORWARD);
    while(myOp.opModeIsActive() && servoBusy(0.43, 0.43)){
      myOp.idle();
    };
  }

  // check for a change in potentiometer returning false if
  // potentiometer stops when servo reaches its set position
  // use in a while loop call getServoPot before entering while
  public boolean servoBusy(double sclf, double scrf) { // reference state
    boolean cmp = true;
    ElapsedTime timer = new ElapsedTime();
    myOp.sleep(5); // wait for servo to move
    // reference pot goal
    double reflf = cmdPot(sclf); // convert servo cmd to est pot voltages

    while (myOp.opModeIsActive() && cmp) {
      // state of pot
      boolean lf_f = false, rf_f = false;
      getServoPot(); // read xxsa global variables
      // error
      if (Math.abs(reflf - lfsa) < 0.1) lf_f = true;
      if (Math.abs(reflf - rfsa) < 0.1) rf_f = true;
      if (lf_f && rf_f) cmp = false;
    }
    // compare to saved  if a servo has settled otherwise return
    return cmp;
  } // end servoBusy

  public void getServoPot() {
    lfsa = lfServo.getVoltage();
    rfsa = rfServo.getVoltage();
  }

  // returns servo cmd set position 0 - 1
  public double potCmd(double pot) {
    double averMax = (lfServo.getMaxVoltage() + rfServo.getMaxVoltage()) / 2.0;
    return pot / averMax;
  }

  // returns estimated potentiometer voltage from
  // servo cmd 0 - 1
  private double cmdPot(double cmd) {
    return (2.3373 * cmd) + 0.455;
  }

  public void driveSpeed(double desV) {
    // Use existing function to drive wheels.
    // left drive

    lfDrive.setPower(desV);

    rfDrive.setPower(desV);
  }

  public void driveStop() {
    lfDrive.setPower(0.0);
    rfDrive.setPower(0.0);
  }

  // calculate distance in turn in encoder pulses as whole pulse; integer
  public int encCntR(int angle) { // encoder count radius turning
    return (int) (gearRatio * mEnc * angle / 360.0);
  }

  // calculate distance to travel in encoder pulses
  public int encCntD(double dist) { // encoder count distance
    return (int) (mEnc * dist / wCir);
  }

  // field centric driving
  public double fieldCentricX(double gmX, double gmY) {
    // xx = xcosB - ysinB
    double botheading = odo.getHeading();
    return gmX * Math.cos(-botheading) - gmY * Math.sin(-botheading);
  }

  // field centric driving
  public double fieldCentricY(double gmX, double gmY) {
    // yy = xsinB + ycosB
    double botheading = odo.getHeading();
    return gmX * Math.sin(-botheading) + gmY * Math.cos(-botheading);
  }

  public double getWDist(double tDist) {
    double vecDist = 0.0;
    return wCir * tDist;
  }

  public double getVecLen(double x, double y) {
    return Math.sqrt((x * x) + (y * y));
  }

  public double getVecAng(double x, double y) {
    if (x == 0.0) return 0.0;
    return Math.atan(y / x);
  }

  //  (r,θ), write x=rcosθ and y=rsinθ.
  public double getX(double d, double a) {
    return d * Math.cos(a);
  }

  public double getY(double d, double a) {
    return d * Math.sin(a);
  }


    // Get encoder position value lf motor
  public int getlfEncoder() {
    return lfDrive.getCurrentPosition();
  }

  // Get encoder position value rf motor
  public int getrfEncoder() {
    return rfDrive.getCurrentPosition();
  }

  // Get encoder position value rr motor
  private static double getAckermanRatio(double wheelBaseWidth, double trackWidth) {
    // arbritary radii greater than track width
    double ackRadius = 400; // mm
    double ain = Math.atan(wheelBaseWidth / (ackRadius - trackWidth / 2.0));
    double aout = Math.atan(wheelBaseWidth / (ackRadius + trackWidth / 2.0));
    // speed difference should be added
    return aout / ain;
  }
} // end class Swerve Components
