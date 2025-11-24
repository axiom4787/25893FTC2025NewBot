package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="The Keep",group="The Keep")
public class TheKeep extends OpMode {
    private DcMotor FrontL = null;
    private DcMotor FrontR = null;
    private DcMotor BackL = null;
    private DcMotor BackR = null;
    private DcMotor Shooter = null;

    private Servo ballEjector = null;
    private Servo spinIndexer = null;
    private CRServo intake = null;
    private int number = 13;
    private double[] spinPosition = {0.3,0.6,0.1,0.14,0.17,0.21,0.25,0.28,0.32,0.36,0.4,0.43,0.47,0.51,0.55,0.58,0.62,0.66,0.7,0.73,0.77,0.8,0.83,0.87,0.91,0.94,0.98};
    @Override
    public void init() {

        FrontL = hardwareMap.get(DcMotor.class,"FrontL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        Shooter = hardwareMap.get(DcMotor.class,"Shooter");
        intake = hardwareMap.get(CRServo.class,"intake");
        FrontL.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.FORWARD);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);
        ballEjector = hardwareMap.get(Servo.class, "ballEjector");
        spinIndexer = hardwareMap.get(Servo.class, "spinIndexer");
        //These lines setup the motors and servos
        //These lines will set the direction of the motors so 1 power is forward

    } // This initializes all the variables and motors

    @Override
    public void loop() {

        FrontL.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
        FrontR.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
        BackL.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/2);
        BackR.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
        // These lines control the wheel motors

        if (gamepad1.leftBumperWasPressed() && number != 0) number -= 1;
        if (gamepad1.rightBumperWasPressed() && number != 26) number += 1;
        spinIndexer.setPosition(spinPosition[number]);
        if (gamepad1.circle) {
            Shooter.setPower(1);
        } else Shooter.setPower(0);
        if (gamepad1.triangle && number % 2 == 1) {
            ballEjector.setPosition(.3);
        } else ballEjector.setPosition(0);
        if (gamepad1.square) {
            intake.setPower(1);
        } else intake.setPower(0);
        // These lines control the spin indexer
        // This controls the ball ejector
        // These lines control the shooting flywheel
        telemetry.addData("SpinIndexer Position", spinPosition[number]);
        telemetry.update();

    } // This section controls all the motors using the remote control

}

