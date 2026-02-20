package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Boilerplate.RTPAxon;
import org.firstinspires.ftc.teamcode.Boilerplate.ThePlantRobotOpMode;

@TeleOp(name = "Test the super smart servo")
public class TestTheAxonServo extends ThePlantRobotOpMode {
    RTPAxon smartServo;

    @Override
    public void opModeInit() {
        smartServo = new RTPAxon(turretLeft, axonServoEncoder, RTPAxon.Direction.REVERSE);
        smartServo.setRtp(false);
        smartServo.forceResetTotalRotation();
//        smartServo.setPidCoeffs(0.01, 0.0, 0.0);
    }

    @Override
    public void opModeRunLoop() {
        smartServo.update();
        double angle = smartServo.getTotalRotation();

        if (gamepad1.dpad_left && angle > -140) {
            smartServo.setPower(-0.2);
        } else if (gamepad1.dpad_right && angle < 140) {
            smartServo.setPower(0.2);
        } else {
            smartServo.setPower(0.0);
        }

        telemetry.addData("Servo log", smartServo.log());
    }
}