package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Crazy Mode Button")
public class CrazyMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        // TODO: Put your real motor setup here
        waitForStart();

        while (opModeIsActive()) {

            // Normal joystick values
            double normalY = -gamepad1.left_stick_y;  // forward/back (push up = positive)
            double normalX = gamepad1.left_stick_x;   // strafe left/right

            double finalForward;
            double finalStrafe;

            // CRAZY MODE CHECK!
            if (gamepad1.a) {
                // A is pressed → SWAP X and Y!! (crazy mode ON)
                finalForward = normalX;   // left/right joystick now controls forward/back
                finalStrafe  = normalY;   // forward/back joystick now controls left/right
                telemetry.addData("MODE", "CRAZY MODE ACTIVE!!!");
            } else {
                // A not pressed → everything normal
                finalForward = normalY;
                finalStrafe  = normalX;
                telemetry.addData("MODE", "Normal (boring) Mode");
            }

            // You can also get turn (twist) normally
            double turn = gamepad1.right_stick_x;

            // TODO: Use these values for your robot drive

            // Show the driver what's happening
            telemetry.addData("Forward Power", finalForward);
            telemetry.addData("Strafe Power", finalStrafe);
            telemetry.update();
        }
    }
}