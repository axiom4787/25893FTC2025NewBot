package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private boolean isFollowingPath = false;

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Globals.autoEndPose);

        Hardware.init(hardwareMap);
        hood = new Hood();
        intake = new Intake();
        vision = new Vision();
        shooter = new Shooter();
        turret = new Turret();

        vision.start();

        allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        allHubs.forEach(hub -> hub.setConstant(0x8000FF));

        telemetry.setMsTransmissionInterval(100);

        selectAlliance();
        Globals.setAlliance(alliance);

        ElapsedTime timer = new ElapsedTime();

        follower.startTeleOpDrive(false);

        while (opModeIsActive()) {
            timer.reset();

            allHubs.forEach(LynxModule::clearBulkCache);

            follower.update();

            double forward = signedSquare(-gamepad1.left_stick_y);
            double right = signedSquare(gamepad1.left_stick_x);
            double turn = signedSquare(gamepad1.right_stick_x);
            boolean isMoving = right != 0 || forward != 0 || turn != 0;

            if (gamepad1.y) {
                follower.setPose(Globals.Misc.CORNER_RELOCALIZE);
            } // drive to the corner to relocalize

            if (gamepad1.left_trigger_pressed) {
                intake.intake();
            } else if (gamepad1.right_trigger_pressed) {
                if (shooter.getError() <= 60) {
                    intake.index();
                } else {
                    intake.off();
                }
            } else if (gamepad1.left_bumper) {
                intake.reverse();
            } else {
                intake.off();
            }

            if (gamepad1.right_trigger_pressed) {
                shooter.shoot();
            } else if (isMoving) {
                shooter.off();
            } else if (gamepad1.right_bumper) {
                shooter.reverse();
            } else {
                shooter.idle();
            }

            shooter.update(follower);
            turret.update(follower);
            hood.update(follower);

            // Vision.update expects heading in degrees; follower.getHeading() is radians.
            vision.update(Math.toDegrees(follower.getHeading()));
            vision.updatePose(follower);

            follower.setTeleOpDrive(-forward, right, -turn, false, isRedAlliance() ? Math.PI : 0);

            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            double h = Math.toDegrees(follower.getHeading());

            telemetry.addData("Alliance", alliance.name());
            telemetry.addData("Robot X | Y | H", "%4.2f | %4.2f | %4.2f", x, y, h);
            telemetry.addData("looptime (ms)", timer.milliseconds());
            telemetry.addLine();
            telemetry.addData("Turret angle", turret.getCurrentAngle());
            telemetry.addData("Turret target", turret.getTargetAngle());
            telemetry.addData("Turret raw power", turret.turretController.getPower());
            telemetry.addLine();
            telemetry.addData("Shooter vel", shooter.getVelocity());
            telemetry.addData("Shooter target", shooter.getTargetVelocity());
            telemetry.addData("Shooter error", shooter.getError());
            telemetry.addLine();
            telemetry.addData("Hood angle", hood.getAngle());
            telemetry.addLine();
            telemetry.update();
        }
    }

    private double signedSquare(double n) {
        return Math.signum(n) * Math.pow(n, 2);
    }
}
