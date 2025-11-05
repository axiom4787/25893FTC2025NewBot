package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="The Keep",group="The Keep")
public class TheKeep extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor shooter = null;
    private Servo ballEjector = null;
    private CRServo spinIndexer = null;
    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        ballEjector = hardwareMap.get(Servo.class, "ballEjector");
        spinIndexer = hardwareMap.get(CRServo.class, "spinIndexer");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        //These lines setup the motors and servos

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //These lines will set the direction of the motors so 1 power is forward

    } // This initializes all the variables and motors

    @Override
    public void loop() {

        leftDrive.setPower((gamepad1.left_stick_y-gamepad1.left_stick_x)/2);
        rightDrive.setPower((gamepad1.left_stick_y+gamepad1.left_stick_x)/2);
        // These lines control the wheel motors

        ballEjector.setPosition(gamepad1.right_stick_y);
        // This controls the ball ejector

        spinIndexer.setPower(gamepad1.left_trigger-gamepad1.right_trigger); //
        // This controls the fidget tech

        if (gamepad1.dpad_up) shooter.setPower(1);
        if (gamepad1.dpad_down) shooter.setPower(0);
        // These lines control the shooting flywheel

    } // This section controls all the motors using the remote control

}
