package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class turboButton extends OpMode {

    public void init() {
        telemetry.addData("Status", "Crazy Mode READY :D");
        telemetry.update();
    }
    public void loop(){

        double y = gamepad1.left_stick_y;
        double forwardSpeed = 0;
        if (!gamepad1.a) {
            forwardSpeed = y * 0.5;
        }


        telemetry.addData("TurboButton", gamepad1.a ? "On" : "Off");
        telemetry.addData("ForwardSpeed", forwardSpeed);
        telemetry.update();
    }
    }






