package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Eater;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.YeeterKing;

@TeleOp(name="Gamepad Driving")
public class GamepadDriveTeleOp extends OpMode {
    MecanumDrive drive = new MecanumDrive(telemetry);
    Eater eater = new Eater();

    YeeterKing yeeter = new YeeterKing();

    @Override
    public void init() {
        drive.init(hardwareMap);
        eater.init(hardwareMap);
        yeeter.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.rightBumperWasReleased()) {
            boolean eaterStatus = eater.toggle();
            telemetry.addData("Eater (Intake) Status", eaterStatus);
        }

        if (gamepad1.yWasReleased()) {
            yeeter.launch(gamepad1.yWasPressed(), 67);
        }
        double forward = -gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, right, rotate);
        telemetry.update();
    }
}
