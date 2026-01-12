package org.firstinspires.ftc.teamcode.members.adishesh;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Chapter3 extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void loop() {
    chapter3();
    chapter4();
    }

    private void chapter3() {
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Is B button pressed", gamepad1.b);
        telemetry.addData("Difference between left and right joysticks y", gamepad1.left_stick_y - gamepad1.right_stick_y);
        telemetry.addData("Sum of triggers", gamepad1.left_trigger + gamepad1.right_trigger);
    }

    private void chapter4() {
         double forwardSpeed;
         if (gamepad1.a) {
             forwardSpeed = gamepad1.left_stick_y;
         } else {
             forwardSpeed = gamepad1.left_stick_y * 0.5;
         }
        telemetry.addData("Forward speed", forwardSpeed);

        if (!gamepad1.a) {
            telemetry.addData("value of x", gamepad1.left_stick_y);
            telemetry.addData("value of y", gamepad1.left_stick_x);
        } else {
            telemetry.addData("value of x", gamepad1.left_stick_x);
            telemetry.addData("value of y", gamepad1.left_stick_y);
        }
    }


}
