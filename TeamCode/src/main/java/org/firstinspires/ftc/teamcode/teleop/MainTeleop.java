package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MainTeleOp", group = "AA_main")
public class MainTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = new Bot(hardwareMap, telemetry, gamepad1, gamepad2);

        bot.teleopInit();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick();
        }
    }
}