package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Test extends OpMode {
    public static Follower follower;
    private PIDController controller;
    private TelemetryManager telemetryM;
    public static double p = 0.2, i = 0.05, d = 0;
    public static double f = 0.0265;
    public static double target = 0;
    private static double vel = 0;
    public static double alpha = 0.6;
    InterpLUT controlPoints = new InterpLUT();
    private DcMotorEx shooterb, shootert, intake;
    private VoltageSensor volt;

    private final int shooterX = 135;
    private final int shooterY = 135;
    Servo latch;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        shooterb = hardwareMap.get(DcMotorEx.class, "sb");
        shootert = hardwareMap.get(DcMotorEx.class, "st");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        volt = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        controlPoints.add(31 / 2.2, 325);
        controlPoints.add(88 / 2.2, 360);
        controlPoints.add(132 / 2.2, 410);
        controlPoints.add(90, 480);
        controlPoints.createLUT();

        latch = hardwareMap.servo.get("latch");
    }

    @Override
    public void start() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.startTeleopDrive();
        follower.update();
        controller = new PIDController(p, i, d);
    }

    /**
     * This updates the robot's pose estimate, the simple bnbnmecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        if (gamepad1.y) {
            latch.setPosition(1);
        } else {
            latch.setPosition(0);
        }

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getPose().getHeading();
        double distance = Math.sqrt(Math.pow(Math.abs(shooterX - robotX), 2) + Math.pow(Math.abs(shooterY - robotY), 2));
//        double theta = Math.atan(Math.abs(shooterY - robotY) / Math.abs(shooterX - robotX)); // radians
//        double turretAngle = theta - robotHeading;
//        double turretDegrees = Math.toDegrees(turretAngle);
//        turretDegrees = (turretDegrees + 180) % 360 / 360.0;
//        target = turretDegrees * 1600 - 600;
//        controller.setPID(p, i, d);
//        int pos = turret.getCurrentPosition();
//        double pid = controller.calculate(pos, target);
//        turret.setPower(pid);

        intake.setPower(gamepad1.right_trigger);
        target = controlPoints.get(distance);
        controller.setPID(p, i, d);
        double presentVoltage = volt.getVoltage();
        vel = vel * alpha + shooterb.getVelocity() * (2 * Math.PI / 28) * (1 - alpha);
        double pid = controller.calculate(vel, target);
        pid = Math.max(-presentVoltage, Math.min(pid, presentVoltage));
        shooterb.setPower((pid + f * target) / presentVoltage);
        shootert.setPower((-1) * (pid + f * target) / presentVoltage);

        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

//        telemetry.addData("Turret angle: ", Math.toDegrees(turretAngle));
        telemetry.addData("Distance: ", distance);
        telemetry.addData("x: ", robotX);
        telemetry.addData("y: ", robotY);
        telemetry.addData("RPM: ", controlPoints.get(distance));

        telemetry.update();
    }
}