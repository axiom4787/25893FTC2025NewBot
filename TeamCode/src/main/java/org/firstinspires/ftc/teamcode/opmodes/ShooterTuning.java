package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.GamepadHelper;

@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuning extends OpMode {
    private Shooter shooter;
    private GamepadHelper gp1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter(hardwareMap);
        gp1 = new GamepadHelper();

        telemetry.addLine("Shooter Tuning Ready");
        telemetry.addLine("RB: Spin Up | LB: Abort | A: Fire");
        telemetry.addLine("DPad Up/Down: Adjust TARGET_RPM");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gp1.risingEdge(GamepadHelper.RB, gamepad1.right_bumper)) {
            shooter.spinUp();
        }
        if (gp1.risingEdge(GamepadHelper.LB, gamepad1.left_bumper)) {
            shooter.abort();
        }
        if (gp1.risingEdge(GamepadHelper.A, gamepad1.a)) {
            shooter.fire();
        }

        if (gp1.risingEdge(GamepadHelper.DPAD_UP, gamepad1.dpad_up)) {
            Shooter.TARGET_RPM += 100;
        }
        if (gp1.risingEdge(GamepadHelper.DPAD_DOWN, gamepad1.dpad_down)) {
            Shooter.TARGET_RPM -= 100;
        }

        shooter.update();

        telemetry.addData("State", shooter.getState());
        telemetry.addData("Target RPM", "%.0f", Shooter.TARGET_RPM);
        telemetry.addData("Filtered RPM", "%.0f", shooter.getRPM());
        telemetry.addData("Raw RPM", "%.0f", shooter.getRawRPM());
        telemetry.addData("Error", "%.0f", shooter.getError());
        telemetry.addData("Acceleration", "%.0f", shooter.getAcceleration());
        telemetry.addData("Voltage", "%.2f", shooter.getVoltage());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.update();
    }
}
