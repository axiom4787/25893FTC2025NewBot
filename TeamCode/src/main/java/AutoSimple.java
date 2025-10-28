package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name = "AutoSimple")
public class AutoSimple {
    int counter;
    int counter_loop;
    @Override
    public void runOpMode() {
        DcMotor front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor launcher_wheel = hardwareMap.get(DcMotor.class, "launcher_wheel");
        CRServo right_launch_servo = hardwareMap.get(CRServo.class, "right_launch_servo");
        CRServo left_launch_servo = hardwareMap.get(CRServo.class, "left_launch_servo");



    }

    private void launch_loop() {
        counter += 1;
        launcher_wheel.setPower(0.6);
        if (counter == 200) {
            right_launch_servo.setPower(-1);
            left_launch_servo.setPower(1);
        }
        if (counter == 280) {
            right_launch_servo.setPower(0);
            left_launch_servo.setPower(0);
        }
        if (counter == 330) {
            right_launch_servo.setPower(-1);
            left_launch_servo.setPower(1);
            counter = 201;
        }
    }
}
