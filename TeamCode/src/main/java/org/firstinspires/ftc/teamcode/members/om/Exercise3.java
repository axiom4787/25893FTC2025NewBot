package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode illustrates all the exercises from Section 3.3.
 */

@TeleOp(name="Om - Exercise 3", group="Exercises")
public class Exercise3 extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y", gamepad1.right_stick_y);

            telemetry.addData("B Button Pressed", gamepad1.b);

            double leftY = gamepad1.left_stick_y;
            double rightY = gamepad1.right_stick_y;
            double difference = Math.abs(leftY - rightY);
            telemetry.addData("Left-Right Y Difference", difference);

            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;
            double sumTriggers = leftTrigger + rightTrigger;
            telemetry.addData("Trigger Sum", sumTriggers);

            telemetry.update();
        }
    }
}
