package org.firstinspires.ftc.teamcode.members.bala;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bala Exercises")

public class Exercises extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {


            //check if b is pressed
            telemetry.addData("Right stick y", gamepad1.right_stick_y);
            telemetry.addData("Is B pressed?", gamepad1.b);
            //telemetry.update();

            double difference = gamepad1.left_stick_y - gamepad1.right_stick_y;
            double sum = gamepad1.left_trigger + gamepad1.right_trigger;

            telemetry.addData("Subtract", difference);
            telemetry.addData("Add", sum);
            telemetry.update();
        }
    }
}