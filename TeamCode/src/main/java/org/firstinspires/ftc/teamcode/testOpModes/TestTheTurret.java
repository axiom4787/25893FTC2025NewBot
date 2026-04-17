package org.firstinspires.ftc.teamcode.testOpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.CachedRTPAxon;
import org.firstinspires.ftc.teamcode.util.Hardware;

@TeleOp
@Disabled
public class TestTheTurret extends LinearOpMode {
    CachedRTPAxon turret;
    public static double kF = 0.05;
    TelemetryManager t;

    public void runOpMode() {
        Hardware.init(hardwareMap);
        turret = Hardware.getTurretServo();

        t = PanelsTelemetry.INSTANCE.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            turret.setPIDFCoefficients(0, 0, 0, kF);
            turret.setPower(kF);
            t.addData("t", turret.getPower());
            t.update();
        }
     }
}
