package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.Globals.Poses.*;
import static org.firstinspires.ftc.teamcode.util.Globals.*;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.LinearOpModeWithAlliance;

import java.util.List;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpModeWithAlliance {
    private Hood hood;
    private Intake intake;
    private Shooter shooter;
    private Vision vision;
    private Turret turret;

    public Follower follower;

    private final PIDFCoefficients headingPIDFCoefficients = Constants.followerConstants.coefficientsHeadingPIDF;
    private final PIDFController headingPIDFController = new PIDFController(headingPIDFCoefficients);
    boolean headingLock = false;

    private List<LynxModule> allHubs;
    private ElapsedTime loopTimer = new ElapsedTime();

    private ElapsedTime visionTimer = new ElapsedTime();

    private boolean shooterEnabled = true;
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

        allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(hub -> {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.setConstant(0x8000FF);
        });

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        selectAlliance();
        Globals.setAlliance(alliance);

        follower.startTeleOpDrive(false);

        loopTimer.reset();
        visionTimer.reset();

        while (opModeIsActive()) {
            allHubs.forEach(LynxModule::clearBulkCache);

            follower.update();
            Globals.updateRobotLocation(follower.getPose());

            double forward = normalizeDrive(-gamepad1.left_stick_y);
            double right = normalizeDrive(gamepad1.left_stick_x);
            double turn = normalizeDrive(gamepad1.right_stick_x);
            boolean isMoving = right != 0 || forward != 0 || turn != 0;

            if (turn != 0 && headingLock) {
                headingLock = false;
            }

            if (gamepad1.bWasPressed()) {
                headingLock = true;
            }

            if (headingLock) {
                double error = MathFunctions.normalizeAngleSigned(heading(GATE_INTAKE) - follower.getHeading());
                headingPIDFController.updateError(error);
                turn = -headingPIDFController.run();
            } // use a PID controller to auto turn to the correct heading for gate intaking

            if (gamepad1.y) {
                follower.setPose(pose(PARK).mirror());
            } // drive to the opponent's parking zone to relocalize

            if (gamepad1.xWasPressed()) shooterEnabled = !shooterEnabled;

            boolean isNearLaunchZone = Globals.isNearLaunchZone();

            if (gamepad1.left_trigger_pressed) {
                intake.intake();
            } else if (gamepad1.right_trigger_pressed) {
                if (isNearLaunchZone) {
                    intake.index();
                } else {
                    intake.off();
                }
            } else if (gamepad1.left_bumper) {
                intake.reverse();
            } else {
                intake.off();
            }

            if (!shooterEnabled) {
                shooter.off();
            } else if (gamepad1.right_bumper) {
                shooter.reverse();
            } else if (isNearLaunchZone) {
                shooter.shoot();
            } else {
                shooter.idle();
            }

            shooter.update();
            turret.update();
            hood.update();

            vision.update(); // Relocalize periodically

            follower.setTeleOpDrive(-forward, right, -turn, false, heading(DRIVE_OFFSET));

            log();

            loopTimer.reset();
        }
    }

    private void log() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getHeading());

        telemetry_addData("Alliance", getAlliance());
        telemetry.addData("Robot X | Y | H", "%4.2f | %4.2f | %4.2f", x, y, h);
        telemetry_addData("looptime (ms)", getLoopTime());
        telemetry.addLine();
        telemetry.addData("Turret angle", turret.getCurrentAngle());
        telemetry.addData("Turret target", turret.getTargetAngle());
        telemetry.addData("Turret raw power", turret.turretController.getPower());
        telemetry_addData("Turret error", getTurretError());
        telemetry.addLine();
        telemetry.addData("Shooter vel", shooter.getVelocity());
        telemetry.addData("Shooter target", shooter.getTargetVelocity());
        telemetry_addData("Shooter error", getShooterError());
        telemetry.addLine();
        telemetry.addData("Hood angle", hood.getAngle());
        telemetry.addLine();
        telemetry.addData("Vision trust", vision.trust);
        telemetry.addLine();
        telemetry_addData("Distance to launch zone", getDistToZone());
        telemetry.update();
    }

    private double normalizeDrive(double n) {
        if (n == 0) return 0;

        double ff = 0.02;

        double squared = Math.signum(n) * Math.pow(n, 2);
        return ff + (1 - ff) * squared;
    }

    private String getAlliance() {
        if (isRedAlliance()) {
            return color("Red", "ef5350");
        } else {
            return color("Blue", "448aff");
        }
    }

    private String getDistToZone() {
        double dist = Globals.distToLaunchZone();

        String color;
        if (dist < 1) {
            color = "00ff00";
        } else if (dist < 8) {
            color = "ffff00";
        } else {
            color = "ff0000";
        }

        String str = String.format("%3.2f", dist);
        return color(str, color);
    }

    private String getLoopTime() {
        double ms = loopTimer.milliseconds();

        String color;
        if (ms < 5) {
            color = "00ff00";
        } else if (ms < 10) {
            color = "ffff00";
        } else {
            color = "ff0000";
        }

        String str = String.format("%3.2f", loopTimer.milliseconds());
        return color(str, color);
    }

    private String getShooterError() {
        double error = shooter.getError();

        if (error < -20) {
            return color(error, "0000ff");
        } else if (error <= 20) {
            return color(error, "00ff00");
        } else if (error <= 60) {
            return color(error, "ffff00");
        } else {
            return color(error, "ff0000");
        }
    }

    private String getTurretError() {
        double error = turret.getTargetAngle() - turret.getCurrentAngle();

        String color;
        if (error < 2) {
            color = "00ff00";
        } else if (error < 5) {
            color = "ffff00";
        } else {
            color = "ff0000";
        }

        String str = String.format("%3.2f", error);
        return color(str, color);
    }

    private String color(Object thing, String color) {
        return "<font color='#" + color + "'>" + thing + "</font>";
    }

    // telemetry.addData does not approve of colors, so we use this
    private void telemetry_addData(String caption, String value) {
        String sep = telemetry.getCaptionValueSeparator();

        telemetry.addLine(caption + sep + value);
    }
}
