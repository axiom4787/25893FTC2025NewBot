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
        smartServo.setRtp(true);
        smartServo.forceResetTotalRotation();
        smartServo.setPidCoeffs(0.05, 0.0, 0.001);
    }

    @Override
    public void opModeRunLoop() {
        smartServo.update();

        if (gamepad1.dpad_left) {
            smartServo.setTargetRotation(-120);
        } else if (gamepad1.dpad_right) {
            smartServo.setTargetRotation(120);
        } else if (gamepad1.circle) {
            smartServo.setRtp(false);
            smartServo.setPower(1);
        }

//        if (gamepad1.dpad_left && angle > -140) {
//            smartServo.setTargetRotation(-60);
//        } else if (gamepad1.dpad_right && angle < 140) {
//            smartServo.setTargetRotation(60);
//        } else {
////            smartServo.setPower(0.0);
//        }

        telemetry.addData("Servo log", smartServo.log());
    }
}