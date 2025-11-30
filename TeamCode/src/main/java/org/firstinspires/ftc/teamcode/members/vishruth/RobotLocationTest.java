package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RobotLocationTest extends OpMode {
    RobotLocation location;
    @Override
    public void init() {
        location = new RobotLocation(0,0,0);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) {
            location.changeY(0.1);
        }
        if(gamepad1.dpad_down) {
            location.changeY(-0.1);
        }
        if(gamepad1.dpad_right) {
            location.changeX(0.1);
        }
        if(gamepad1.dpad_left) {
            location.changeX(-0.1);
        }

        telemetry.addData("Robot Location",location.toString());
        telemetry.update();
    }
}
