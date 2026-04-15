package org.firstinspires.ftc.teamcode.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Boilerplate.ThePlantRobotOpMode;

@TeleOp(name = "test the smart shooter")
@Disabled
public class TestTheSmartShooter extends ThePlantRobotOpMode {
    TelemetryManager panelsTelemetry;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    @Override
    public void opModeInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void opModeRunLoop() {
        if (gamepad1.right_bumper) {
            smartShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
        if (gamepad1.circle) smartShooter.setVelocity(1000);
        if (gamepad1.cross)  smartShooter.setVelocity(1800);
        if (gamepad1.triangle) smartShooter.setPower(0);

        panelsTelemetry.addData("shooter velocity", smartShooter.getVelocity());
        panelsTelemetry.update(telemetry);
    }
}
