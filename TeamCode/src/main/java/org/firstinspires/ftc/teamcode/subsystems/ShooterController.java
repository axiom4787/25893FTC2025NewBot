package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class ShooterController {
    public enum State { IDLE, SPIN_UP, FEED, SPIN_DOWN, EJECT }

    private final DcMotor intake, intake2;
    private final DcMotorEx launcher;

    // Tunables
    private final int shortShotVelocity = 1400; // spin power
    private final int longShotVelocity = 2000; // spin power
    private final int ejectVelocity = 1000;
    private final double targetPower = 1.0; // spin power
    private final double intakePower = 1.0; // spin power
    private final long spinUpMs = 2000; // wait time before feed
    private final long feedMs = 1000; // time to press the ball
    private final long spinDownMs = 1000; // optional coast-down window
    private final int velocityTolerance = 35; // how far away from the target velocity is OK
    private String shotType = "";

    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;
    private boolean busy = false;
    private int numberOfShots = 0;
    private int numberOfBalls = 0;
    private double lastVelocity = 0;


    public ShooterController(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        // Encoder logic -------------------------------------------------
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set motor directions (adjust if movement is inverted) ----------
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Set motor behavior ----------------------------------------------
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /** Start a single, timed shot. Returns immediately (non-blocking). */
    public void startShot(int shots, String type) {
        if(busy) {
            //telemetry.addLine("Shooter is busy");
            //telemetry.update();
            return; // ignore if already running
        }
        numberOfShots = shots;
        shotType = type;
        //telemetry.addLine("Shot Initiated");
        //telemetry.update();
        busy = true;
        state = State.SPIN_UP;
        timer.reset();

        //launcher.setPower(targetPower);
        if (Objects.equals(shotType, "short")){
            launcher.setVelocity(shortShotVelocity);
        } else {
            launcher.setVelocity(longShotVelocity);
        }
    }

    /** Call this every loop to advance the sequence without blocking. */
    public void update() {
        switch (state) {
            case IDLE:
                // nothing
                break;

            case SPIN_UP:
                lastVelocity = launcher.getVelocity();
                if (shotType == "short"){
                    if (Math.abs(shortShotVelocity - launcher.getVelocity()) < velocityTolerance) {
                    //if (timer.milliseconds() >= spinUpMs) {
                        state = State.FEED;
                        timer.reset();
                        intake.setPower(intakePower); // push ball into shooter
                        intake2.setPower(0);
                    }
                } else {
                    if (Math.abs(longShotVelocity - launcher.getVelocity()) < velocityTolerance) {
                    //if (timer.milliseconds() >= spinUpMs) {
                        state = State.FEED;
                        timer.reset();
                        intake.setPower(intakePower); // push ball into shooter
                        intake2.setPower(0);
                    }
                }
                
                break;

            case EJECT:
                lastVelocity=  launcher.getVelocity();
                if (Math.abs(ejectVelocity - launcher.getVelocity()) < velocityTolerance) {
                // if (timer.milliseconds() >= spinUpMs) {
                    state = State.FEED;
                    timer.reset();
                    intake.setPower(intakePower); // push ball into shooter
                    intake2.setPower(0);
                }
                break;

            case FEED:
                if (timer.milliseconds() >= feedMs) {
                    state = State.SPIN_DOWN;
                    timer.reset();
                    launcher.setVelocity(0);
                    intake.setPower(0);
                }
                break;

            case SPIN_DOWN:
                if (numberOfShots > 1){
                    state = State.SPIN_UP;
                    if (shotType == "short") {
                    launcher.setVelocity(shortShotVelocity);
                    } else {
                        launcher.setVelocity(longShotVelocity);
                    }
                    intake2.setPower(intakePower);
                    numberOfShots--;
                    //telemetry.addLine("Shots left:" + numberOfShots);
                }
                if (timer.milliseconds() >= spinDownMs) {
                    state = State.IDLE;
                    busy = false;
                }
                break;
        }
    }
    
    public void eject(int balls) {
        if(busy) { 
            //telemetry.addLine("Shooter is busy");
            return; // ignore if already running
        }
        numberOfBalls = balls;
        //telemetry.addLine("Ejection Initiated");
        busy = true;
        state = State.EJECT;
        timer.reset();

        // launcher.setPower(targetPower);
        launcher.setVelocity(ejectVelocity);
    }

    public void startIntake() {
        intake.setPower(1);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void startIntake2() {
        intake2.setPower(1);
    }

    public void stopIntake2() {
        intake2.setPower(0);
    }

    public void startLauncher() {
        launcher.setPower(1);
    }

    public void stopLauncher() {
        launcher.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Shooter -----");
        telemetry.addData("Shooter Velocity = ", lastVelocity);
    }

    public boolean isBusy() { return busy; }
    public State getState() { return state; }
}
    
