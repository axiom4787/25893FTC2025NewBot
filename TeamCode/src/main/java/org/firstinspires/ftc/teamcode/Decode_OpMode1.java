package org.firstinspires.ftc.teamcode;
import static android.provider.SyncStateContract.Helpers.update;

import android.util.Log;
import android.widget.Switch;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.color_sens_stuff;
import org.firstinspires.ftc.teamcode.util.AprilTagLedManager;
import org.firstinspires.ftc.teamcode.util.ledstuff;
//PLEASEPLEASE
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
public class Decode_OpMode1 extends LinearOpMode {
    private enum intakesys{
        idle,
        intake,
        ready
    }
    private ElapsedTime runtime = new ElapsedTime();
    private Decode_OpMode1.intakesys IOState = Decode_OpMode1.intakesys.idle;
    //color
    color_sens_stuff colorTest = new color_sens_stuff();
    color_sens_stuff.DetectedColor detectedColor;
    private AprilTagLedManager ledManager;

    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    DcMotor flyleft = null;
    DcMotor flyright = null;
    DcMotor intake = null;
    Servo shoot = null;
    double lasttime = 0;
    void update() {
        switch (IOState) {
            case idle:
                intake.setPower(0);
                shoot.setPosition(-0.1);
                flyleft.setPower(0);
                flyright.setPower(0);
                break;

            case intake:
                intake.setPower(1);
                shoot.setPosition(-0.1);
                if (runtime.seconds() - lasttime > 1) {
                    intake.setPower(0);
                    flyright.setPower(0.9);
                    flyleft.setPower(0.9);
                }
                break;

            case ready:
                shoot.setPosition(0.3);
                if (runtime.seconds() - lasttime > 1) {
                    IOState = intakesys.idle;
                    break;
                }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Driving Motors
        frontLeftMotor = hardwareMap.dcMotor.get("LFMotor");
        backLeftMotor = hardwareMap.dcMotor.get("LBMotor");
        frontRightMotor = hardwareMap.dcMotor.get("RFMotor");
        backRightMotor = hardwareMap.dcMotor.get("RBMotor");
        flyleft = hardwareMap.dcMotor.get("fl");
        flyright = hardwareMap.dcMotor.get("fr");
        intake = hardwareMap.dcMotor.get("int");
        shoot = hardwareMap.servo.get("sht");




        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flyright.setDirection(DcMotorSimple.Direction.REVERSE);
        flyleft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //color
        colorTest.init(hardwareMap);

        //camera
        ledManager = new AprilTagLedManager(hardwareMap, "limelight", "rgbLight");


        waitForStart();
        if (isStopRequested()) return;
        double powerMultiplier = 1.0; // Start at full power
        runtime.reset();

        while (opModeIsActive()) {
            update();
            double y = -gamepad2.left_stick_y;   // forward/back
            double x = -gamepad2.left_stick_x;   // strafe
            double rx = -gamepad2.right_stick_x; // turn

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (-y + x + rx) / denominator;
            double backLeftPower = (-y - x + rx) / denominator;
            double frontRightPower = (-y - x - rx) / denominator;
            double backRightPower = (-y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            detectedColor = colorTest.getDetectedColor(telemetry);
            telemetry.addData("Color",detectedColor);
            ledManager.update();

            if (gamepad1.right_bumper){
                lasttime = runtime.seconds();
                IOState = intakesys.intake;
            }
            if (gamepad1.left_bumper){
                lasttime = runtime.seconds();
                IOState = intakesys.ready;
            }

        }
    }
}