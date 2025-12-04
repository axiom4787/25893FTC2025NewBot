package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class OurTeleOp extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private Servo flap;
    private DcMotor rightDrive;
    private float flyWheelVelocity = 1300;
    private float  ticksPerRev = 288;
    private float positionPerDegree = 1f / 270f;
    private boolean flyWheelPowered;
    private boolean flapUp;
    private boolean feedRollerPowered;
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        flap = hardwareMap.get(Servo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        feedRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedRoller.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        flap.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("a to turn on/off the flywheel");
        telemetry.addLine("b to turn on/off the agitator");
        telemetry.addLine("x to turn on/off the feed roller");
        telemetry.addLine("y to turn off flywheel agitator and set feed roller angle");

        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            basicMovement();
            turnOnMotors();
            flyWheel();
            feedRollerSetter();

            telemetry.addLine("Flywheel Velocity" + flywheel.getVelocity());
            telemetry.update();
        }
    }
    public void basicMovement() {
        float x;
        float y;

        x = gamepad1.right_stick_x;
        y = gamepad1.left_stick_y;
        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }
    public void turnOnMotors() {
        if(gamepad1.aWasPressed()) {
            flyWheelPowered = !flyWheelPowered;
        }
        if(gamepad1.bWasPressed()) {
            // + 30 * positionPerDegree because the starting position is 30 so to set it to a 0 you have to add 30
            if(flapUp) {
                flapUp = false;
                flap.setPosition(1 - (30 * positionPerDegree));
            } else {
                flapUp = true;
                flap.setPosition(1 - ((90 + 30) * positionPerDegree));
            }
        }
        if(gamepad1.yWasPressed()) {
            float angleOff = (feedRoller.getCurrentPosition() % ticksPerRev);

            feedRollerPowered = false;

            feedRoller.setTargetPosition((int)(feedRoller.getCurrentPosition() - angleOff));
            feedRoller.setPower(1);
            feedRoller.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            flywheel.setPower(0);

            flapUp = false;
            flap.setPosition(1 - (30 * positionPerDegree));
        }
    }
    public void feedRollerSetter() {
        double error = Math.abs(flyWheelVelocity - flywheel.getVelocity());
        boolean inRange = error <= 50;
        //50 is a tolerance
        if (!feedRoller.isBusy()) {
            feedRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            feedRollerPowered = inRange;
            feedRoller.setPower(feedRollerPowered ? 1 : 0);
        }
    }
    public void flyWheel() {
        if(flyWheelPowered) {
            flywheel.setVelocity(flyWheelVelocity);
        } else {
            flywheel.setVelocity(0);
        }
    }
}
