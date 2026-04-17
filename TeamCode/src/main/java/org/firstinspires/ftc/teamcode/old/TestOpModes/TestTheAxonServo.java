package org.firstinspires.ftc.teamcode.old.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.old.RTPAxon;
import org.firstinspires.ftc.teamcode.old.ThePlantRobotOpMode;

@TeleOp(name = "Test the super smart servo")
@Disabled
public class TestTheAxonServo extends ThePlantRobotOpMode {
    RTPAxon smartServo;

    @Override
    public void opModeInit() {
        smartServo = new RTPAxon(turretLeft, axonServoEncoder, RTPAxon.Direction.REVERSE);
        smartServo.setRtp(true);
        smartServo.forceResetTotalRotation();
        smartServo.setPidCoeffs(0.015, 0.0, 0.0001);
    }

    @Override
    public void opModeRunLoop() {
        smartServo.update();

        if (gamepad1.dpad_left) {
            smartServo.setTargetRotation(-120);
        } else if (gamepad1.dpad_right) {
            smartServo.setTargetRotation(120);
        } else if (gamepad1.dpad_up) {
            smartServo.setTargetRotation(0);
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