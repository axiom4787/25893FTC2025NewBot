package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name = "TeleOp Main", group = "Competition")
public class TeleOpMain extends OpMode {
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Intake intake;
    private GamepadHelper gp1;
    private GamepadHelper gp2;
    private boolean lastTriggerPressed = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gp1 = new GamepadHelper();
        gp2 = new GamepadHelper();

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        drivetrain.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        boolean triggerPressed = gamepad1.right_trigger > 0.1;
        shooter.setTriggerHeld(triggerPressed);

        if (triggerPressed && !lastTriggerPressed) {
            shooter.spinUp();
        }

        if (!triggerPressed && lastTriggerPressed) {
            shooter.abort();
        }

        if (shooter.isReady() && triggerPressed) {
            shooter.fire();
        }

        lastTriggerPressed = triggerPressed;

        shooter.update();

        boolean intakeOn = gp1.toggle(GamepadHelper.B, gamepad1.b);
        if (intakeOn) {
            intake.run();
        } else {
            intake.stop();
        }

        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("Shooter RPM", "%.0f", shooter.getRPM());
        telemetry.addData("Shooter Ready", shooter.isReady());
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.update();
    }
}
