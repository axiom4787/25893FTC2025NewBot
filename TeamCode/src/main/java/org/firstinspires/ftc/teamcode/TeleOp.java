package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor launcherwheel;
    private CRServo right_launch_servo;
    private CRServo left_launch_servo;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        int counter;

        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        launcherwheel = hardwareMap.get(DcMotor.class, "launcher wheel");
        right_launch_servo = hardwareMap.get(CRServo.class, "right_launch_servo");
        left_launch_servo = hardwareMap.get(CRServo.class, "left_launch_servo");

        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        launcherwheel.setDirection(DcMotor.Direction.FORWARD);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                front_left_motor.setPower(gamepad1.left_stick_y * 0.75 - gamepad1.right_stick_x * 0.5);
                front_right_motor.setPower(gamepad1.left_stick_y * 0.75 + gamepad1.right_stick_x * 0.5);
                if (gamepad1.right_bumper) {
                    launcherwheel.setPower(0.6);
                    counter += 1;
                    if (counter == 200) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                    if (counter == 290) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                    }
                    if (counter == 340) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                        counter = 201;
                    }
                } else {
                    counter = 0;
                    launcherwheel.setPower(0);
                    right_launch_servo.setPower(0);
                    left_launch_servo.setPower(0);
                }
                if (gamepad1.left_bumper) {
                    launcherwheel.setPower(0.7);
                }
            }
        }
    }
}
