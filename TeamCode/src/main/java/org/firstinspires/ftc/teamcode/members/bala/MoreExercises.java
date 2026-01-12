package org.firstinspires.ftc.teamcode.members.bala;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()

public class MoreExercises extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {

            //1.Turbo button
            //If a is pressed speed is multiplied by 1
            //If not, speed is cut in half
            double joystick;
            if (gamepad1.a) {
                joystick = gamepad1.left_stick_y * 1.0;
                telemetry.addData("Forward Speed", joystick);
                telemetry.update();
            } else {
                joystick = gamepad1.left_stick_y * 0.5;
            }


            //2.If a is pressed, x and y are swapped
            //If not, then they stay the same
            if (gamepad1.a) {
                double x = gamepad1.left_stick_y;
                double y = gamepad1.left_stick_x;
                telemetry.addData("gamepad1.left_stick_y", x);
                telemetry.addData("gamepad1.left_stick_x", y);
                telemetry.update();
            } else {
                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y;
                telemetry.addData("gamepad1.left_stick_x", x);
                telemetry.addData("gamepad1.left_stick_y", y);
                telemetry.update();

            }


        }
    }
}