package org.firstinspires.ftc.teamcode.FRLib.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Logger;

public class Launcher {

    final double FEED_TIME = 0.23;
    //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0;
    //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double TIME_BETWEEN_SHOTS = 2;


    ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();

    private TeleOpLaunchStates launchStatesT = TeleOpLaunchStates.IDLE;
    private AutoLaunchStates launchStatesA = AutoLaunchStates.IDLE;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1510;
    final double LAUNCHER_MIN_VELOCITY = 1500;

    double launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY;
    double launcherMinVelocity = LAUNCHER_MIN_VELOCITY;

    DcMotorEx launcher;
    CRServo rightFeeder,leftFeeder;
    Logger logger;

    LinearOpMode opMode;

    public CRServo getLeftFeeder() {
        return leftFeeder;
    }

    public CRServo getRightFeeder() {
        return rightFeeder;
    }

    public DcMotorEx getLauncher() {
        return launcher;
    }

    public enum TeleOpLaunchStates {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private enum AutoLaunchStates {
        IDLE,
        PREPARE,
        LAUNCH,
    }
// Getters
    public double getLauncherTargetVelocity() {
        return launcherTargetVelocity;
    }
    public TeleOpLaunchStates getLaunchStatesT() {
        return launchStatesT;
    }
    public AutoLaunchStates getLaunchStatesA() {
        return launchStatesA;
    }
    public double getLauncherMinVelocity() {
        return launcherMinVelocity;
    }

// Setters

    public void setLauncherMinVelocity(double launcherMinVelocity) {
        this.launcherMinVelocity = launcherMinVelocity;
    }

    public void setLaunchStatesT(TeleOpLaunchStates launchStatesT) {
        this.launchStatesT = launchStatesT;
    }
    public void setLauncherTargetVelocity(double launcherTargetVelocity) {
        this.launcherTargetVelocity = launcherTargetVelocity;
    }

    public Launcher(@NonNull LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void init(){
        logger = new Logger(opMode.telemetry);

        launcher = opMode.hardwareMap.get(DcMotorEx.class, "launcher");
        rightFeeder = opMode.hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder = opMode.hardwareMap.get(CRServo.class, "left_feeder");

        logger.logData(Logger.LoggerMode.STATUS,"Hardware","Created");
        logger.update();

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);

        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        logger.logData(Logger.LoggerMode.STATUS,"Hardware","Initialized");
        logger.update();
    }
// Launching Stuff
    public void launchTeleOp(boolean shotRequested) {
        switch (launchStatesT) {
            case IDLE:
                if (shotRequested) {
                    launchStatesT = TeleOpLaunchStates.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(launcherTargetVelocity);
                if (launcher.getVelocity() > launcherMinVelocity) {
                    launchStatesT = TeleOpLaunchStates.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchStatesT = TeleOpLaunchStates.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME) {
                    launchStatesT = TeleOpLaunchStates.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    public boolean launchAuto(boolean shotRequested) {
        switch (launchStatesA) {
            case IDLE:
                if (shotRequested) {
                    launchStatesA = AutoLaunchStates.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(launcherTargetVelocity);
                if (launcher.getVelocity() > launcherMinVelocity){
                    launchStatesA = AutoLaunchStates.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchStatesA = AutoLaunchStates.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }
   // util

   public void stopFlywheel(){
        launcher.setVelocity(0);
   }

   public void spinUpFlywheel(){
        launcher.setVelocity(launcherTargetVelocity);
   }




}
