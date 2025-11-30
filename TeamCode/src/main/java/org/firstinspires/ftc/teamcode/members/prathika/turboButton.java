package org.firstinspires.ftc.teamcode.members.prathika;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//hello
public class turboButton {
    @TeleOp

    public void loop{

        double x = gamepad1.left_stick_y;
        double forwardSpeed;
        if (!gamepad1.a) {
            forwardSpeed = y * 0.5;
        } else (gamepad1.a) {
                forwardSpeed = y * 1;
        }
        telemetry.addData("TurboButton", gamepad1.a ? "On" : "Off");
        telemetry.addData("ForwardSpeed", forwardSpeed);
        telemetry.update();
    }
}






