package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.LinearOpModeWithAlliance;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpModeWithAlliance {
    private Hood hood;
    private Intake intake;
    private Vision vision;
    private Shooter shooter;
    private Turret turret;

    private Follower follower;

    private final PIDFCoefficients headingPIDFCoefficients = Constants.followerConstants.coefficientsHeadingPIDF;
    private final PIDFController headingPIDFController = new PIDFController(headingPIDFCoefficients);
    boolean headingLock = false;

    private List<LynxModule> allHubs;
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Globals.autoEndPose);

        Hardware.init(hardwareMap);
        hood = new Hood();
        intake = new Intake();
        vision = new Vision(telemetry);
        shooter = new Shooter();
        turret = new Turret();

        vision.start();

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

        while (opModeIsActive()) {
            allHubs.forEach(LynxModule::clearBulkCache);

            follower.update();
            Globals.Zones.updateRobotLocation(follower);

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
                double error = MathFunctions.normalizeAngleSigned(Globals.Misc.GATE_INTAKE.getHeading() - follower.getHeading());
                headingPIDFController.updateError(error);
                turn = -headingPIDFController.run();
            } // use a PID controller to auto turn to the correct heading for gate intaking

            if (gamepad1.y) {
                follower.setPose(Globals.Misc.PARK.mirror());
            } // drive to the opponent's parking zone to relocalize

            boolean isNearLaunchZone = Globals.Zones.isNearLaunchZone();

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

            if (gamepad1.right_bumper) {
                shooter.reverse();
            } else if (isNearLaunchZone) {
                shooter.shoot();
            } else {
                shooter.idle();
            }

            shooter.update(follower);
            turret.update(follower);
            hood.update(follower);

//            // Todo: Make vision actually work
//            if (Math.random() > 0.95) {
//                vision.update();
//                vision.updatePose(follower);
//            }

            follower.setTeleOpDrive(-forward, right, -turn, false, Globals.Misc.FIELD_RELATIVE_DRIVE_HEADING_OFFSET);

            log();

            loopTimer.reset();
        }
    }

    private void log() {
        String sep = telemetry.getCaptionValueSeparator();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getHeading());

        //telemetry.addData("Alliance", allianceColored());
        telemetry.addLine(String.format("Alliance%s%s", sep, allianceColored()));
        telemetry.addData("Robot X | Y | H", "%4.2f | %4.2f | %4.2f", x, y, h);
        //telemetry.addData("looptime (ms)", looptimeColored());
        telemetry.addLine(String.format("looptime (ms)%s%s", sep, looptimeColored()));
        telemetry.addLine();
        telemetry.addData("Turret angle", turret.getCurrentAngle());
        telemetry.addData("Turret target", turret.getTargetAngle());
        telemetry.addData("Turret raw power", turret.turretController.getPower());
        //telemetry.addData("Turret error", turretErrorColored());
        telemetry.addLine(String.format("Turret error%s%s", sep, turretErrorColored()));
        telemetry.addLine();
        telemetry.addData("Shooter vel", shooter.getVelocity());
        telemetry.addData("Shooter target", shooter.getTargetVelocity());
        //telemetry.addData("Shooter error", shooterErrorColored());
        telemetry.addLine(String.format("Shooter error%s%s", sep, shooterErrorColored()));
        telemetry.addLine();
        telemetry.addData("Hood angle", hood.getAngle());
        telemetry.addLine();
        //telemetry.addData("Distance to launch zone", "%3.2f", distToLaunchColored());
        telemetry.addLine(String.format("Distance to launch zone%s%s", sep, distToLaunchColored()));
        telemetry.update();
    }

    private double normalizeDrive(double n) {
        if (n == 0) return 0;

        double ff = 0.02;

        double squared = Math.signum(n) * Math.pow(n, 2);
        return ff + (1 - ff) * squared;
    }

    private String allianceColored() {
        if (isRedAlliance()) {
            return color("Red", "ef5350");
        } else {
            return color("Blue", "448aff");
        }
    }

    private String distToLaunchColored() {
        double dist = Globals.Zones.distToLaunchZone();
        if (dist < 1) {
            return color(dist, "00ff00");
        } else if (dist < 8) {
            return color(dist, "ffff00");
        } else {
            return color(dist, "ff0000");
        }
    }

    private String looptimeColored() {
        double ms = loopTimer.milliseconds();
        if (ms < 5) {
            return color(ms, "00ff00");
        } else if (ms < 10) {
            return color(ms, "ffff00");
        } else {
            return color(ms, "ff0000");
        }
    }

    private String shooterErrorColored() {
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

    private String turretErrorColored() {
        double error = turret.getTargetAngle() - turret.getCurrentAngle();
        if (error < 2) {
            return color(error, "00ff00");
        } else if (error < 5) {
            return color(error, "ffff00");
        } else {
            return color(error, "ff0000");
        }
    }

    private String color(Object thing, String color) {
        return "<font color='#" + color + "'>" + thing + "</font>";
    }
}
