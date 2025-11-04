package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp
public class Shooter extends OpMode {
    private FtcDashboard dashboard;
    private PIDController controller;
    private TelemetryManager telemetryM;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double f = 0.0265;
    public static double target = 0;
    private static double vel = 0;
    public static double alpha = 0.6;

    private DcMotorEx shooterb, shootert, intake;
    private VoltageSensor volt;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(p, i, d);
        shooterb = hardwareMap.get(DcMotorEx.class, "sb");
        shootert = hardwareMap.get(DcMotorEx.class, "st");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.right_trigger);

        controller.setPID(p, i, d);
        double presentVoltage = volt.getVoltage();
        vel = vel * alpha + shooterb.getVelocity() * (2 * Math.PI / 28) * (1 - alpha);
        double pid = controller.calculate(vel, target);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
        shooterb.setPower((pid + f * target) / presentVoltage);
        shootert.setPower((-1) * (pid + f * target) / presentVoltage);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Velocity", vel);
        packet.put("Target", target);
        packet.put("Power", pid);
        dashboard.sendTelemetryPacket(packet);

        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}