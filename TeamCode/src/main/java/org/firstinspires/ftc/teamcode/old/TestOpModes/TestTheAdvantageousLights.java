package org.firstinspires.ftc.teamcode.old.TestOpModes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.old.Config;

@TeleOp(name = "test the lights", group = "test")
@Disabled
public class TestTheAdvantageousLights extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config();
        config.init(hardwareMap);

        RevBlinkinLedDriver lights = config.lights;

//        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                // limelight CANNOT see, IS NOT up to speed
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else if (gamepad1.dpad_right) {
                // limelight CANNOT see, shooter IS up to speed
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (gamepad1.dpad_up) {
                // limelight CAN see, shooter IS NOT up to speed
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (gamepad1.dpad_down) {
                // limelight CAN see, shooter IS up to speed
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                // test???
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }
        }
    }
}
