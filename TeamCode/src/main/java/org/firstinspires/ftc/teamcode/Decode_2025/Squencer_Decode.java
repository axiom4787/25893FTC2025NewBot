package org.firstinspires.ftc.teamcode.Decode_2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Squencer_Decode {
    private LinearOpMode myOp = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    Squencer_Decode(LinearOpMode opmode) {
        myOp = opmode;
    }
    public void gamePad() {
        try {
            myOp.telemetry.addLine("A - 90 deg STRAF");
            myOp.telemetry.addLine("left stick Y drive");
            myOp.telemetry.addLine("right stick Y turn");
            myOp.telemetry.addLine("B  button  toggle");
            myOp.telemetry.addLine(" speed 70% or 100%");
            if (myOp.gamepad1.a) {

            }
            if (myOp.gamepad1.b) {

            }
        } catch (Exception e) {
            myOp.telemetry.addLine(", exception in gamePadTeleOP");
            myOp.telemetry.update();
            myOp.sleep(2000);
            myOp.requestOpModeStop();
        }
    }
    private void debounce(){

    }
}
