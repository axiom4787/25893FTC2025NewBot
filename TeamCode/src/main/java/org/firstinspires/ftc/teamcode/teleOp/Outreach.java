package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.Globals.Paths.*;
import static org.firstinspires.ftc.teamcode.util.Globals.Poses.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.LinearOpModeWithAlliance;

import java.util.List;

@TeleOp
public class Outreach extends LinearOpModeWithAlliance {
    private Hood hood;
    private Intake intake;
    private Shooter shooter;
    private Vision vision;
    private Turret turret;
    private Lights lights;

    public Follower follower;

    private List<LynxModule> allHubs;

    private boolean shooterEnabled = true;
    private boolean turretEnabled = true;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Globals.getSavedPose());

        Context.init(this);
        hood = new Hood();
        intake = new Intake();
        vision = new Vision();
        shooter = new Shooter();
        turret = new Turret();
        lights = new Lights();

        allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(hub -> {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            hub.setConstant(0x8000FF);
        });

        alliance = Globals.getAlliance();
        selectAlliance();
        Globals.setAlliance(alliance);

        follower.startTeleOpDrive(false);

        follower.setStartingPose(new Pose(70.75, 70.75, 0));

        while (opModeIsActive()) {
            follower.update();
            Globals.updateRobotLocation(follower.getPose());

            if (gamepad1.back && gamepad1.y) {
                follower.setHeading(Math.PI - heading(DRIVE_OFFSET));
            } // wall slam to reset heading

            if (gamepad1.back && gamepad1.xWasPressed()) {
                shooterEnabled = !shooterEnabled;
            } // toggle shooter

            if (gamepad1.back && gamepad1.aWasPressed()) {
                if (turretEnabled) {
                    turretEnabled = false;
                } else {
                    turretEnabled = true;
                    turret.turretController.forceResetTotalRotation();
                }
            } // toggle manual turret

            if (!turretEnabled) {
                if (gamepad1.dpad_left) {
                    turret.turretController.setPower(-0.5);
                } else if (gamepad1.dpad_right) {
                    turret.turretController.setPower(-0.5);
                } else {
                    turret.turretController.setPower(0);
                }
                turret.turretController.update();
            }

            boolean isNearLaunchZone = Globals.isNearLaunchZone();

            if (gamepad1.right_trigger_pressed) {
                if (Globals.isInLaunchZone() && turret.getAbsError() < 15) {
                    intake.index();
                } else {
                    intake.off();
                }
            } else if (gamepad1.left_trigger_pressed) {
                intake.intake();
            } else if (gamepad1.left_bumper) {
                intake.reverse();
            } else {
                intake.off();
            }

            if (!shooterEnabled) {
                shooter.off();
            } else if (isNearLaunchZone) {
                shooter.shoot();
            } else {
                shooter.idle();
            }

            if (gamepad1.right_trigger_pressed && Globals.isInLaunchZone()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            } else if (gamepad1.left_trigger_pressed) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            } else if (Globals.isInLaunchZone()) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                RevBlinkinLedDriver.BlinkinPattern allianceColor = isRedAlliance()
                        ? RevBlinkinLedDriver.BlinkinPattern.RED
                        : RevBlinkinLedDriver.BlinkinPattern.BLUE;
                lights.setPattern(allianceColor);
            }

            intake.update();
            shooter.update();
            turret.update();
            hood.update();
            vision.update();

            double forward = normalizeDrive(-gamepad1.left_stick_y);
            double right = normalizeDrive(gamepad1.left_stick_x);
            double turn = normalizeDrive(gamepad1.right_stick_x);
            follower.setTeleOpDrive(-forward, right, -turn, false, heading(DRIVE_OFFSET));

            if (gamepad1.back) {
                logDebug();
            } else {
                log();
            }
        }
    }

    private void log() {
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("Left stick: Drive");
        telemetry.addLine("Right stick: Turn");
        telemetry.addLine();
        telemetry.addLine("Left trigger: Intake");
        telemetry.addLine("Right trigger: Shoot");
        telemetry.update();
    }

    private void logDebug() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        telemetry.addLine("--- Warnings ---");
        telemetry.addLine();
        telemetry.addLine("--- end warnings ---");
        telemetry.update();
    }

    private double normalizeDrive(double n) {
        if (n == 0) return 0;

        double max = 1;
        double ff = 0.02;

        double squared = max * Math.signum(n) * Math.pow(n, 2);
        return ff + (max - ff) * squared;
    }
}