package org.firstinspires.ftc.teamcode.members.prathika;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class CrazyButton extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            if (gamepad1.a) {
                double temp = x;
                x = y;
                y = temp;
            }
            telemetry.addData("Crazy mode", gamepad1.a ? "On" : "off");
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.update();
        }
    }
}

