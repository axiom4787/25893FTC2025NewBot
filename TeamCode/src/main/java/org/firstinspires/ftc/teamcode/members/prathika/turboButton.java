package org.firstinspires.ftc.teamcode.members.prathika;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Turbo Button Prathika")
public class turboButton extends LinearOpMode {
    @Override
    public void runOpMode() {
       waitForStart();

        while (opModeIsActive()) {

            double x = gamepad1.left_stick_y;
            double forwardSpeed;
            if (!gamepad1.a) {
                forwardSpeed = x * 0.5;
            } else {
                forwardSpeed = x * 1;
            }
            telemetry.addData("TurboButton", gamepad1.a ? "On" : "Off");
            telemetry.addData("ForwardSpeed", forwardSpeed);
            telemetry.update();
        }
    }
}





