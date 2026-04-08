package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Boilerplate.LinearOpModeWithAlliance;
import org.firstinspires.ftc.teamcode.NewAutos.Shared2;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "another one?")
public class Teleop extends LinearOpModeWithAlliance {
    private DriveSubsystem driveSubsystem;
    private HoodSubsystem hoodSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LimeLightSubsystem limeLightSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private TurretSubsystem turretSubsystem;

    private Follower follower;
    private final boolean usePedroDrive = true;
    private boolean isFollowingPath = false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Shared2.autoEndPose);

        Hardware.init(hardwareMap);
        driveSubsystem = new DriveSubsystem();
        hoodSubsystem = new HoodSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        limeLightSubsystem = new LimeLightSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        turretSubsystem = new TurretSubsystem();

        super.selectAlliance();
        Shared2.setAlliance(alliance);

        ElapsedTime timer = new ElapsedTime();

        if (usePedroDrive) follower.startTeleOpDrive(false);

        while (opModeIsActive()) {
            timer.reset();

            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            boolean isMoving = right != 0 || forward != 0 || turn != 0;

            telemetry.addData("shared alliance", Shared2.getAlliance().name());

            if (gamepad1.left_trigger_pressed) {
                // intake
                intakeSubsystem.intake();
            } else if (gamepad1.right_trigger_pressed) {
                // shoot
                if (shooterSubsystem.getTargetVelocity() - shooterSubsystem.getVelocity() < 40) {
                    intakeSubsystem.index();
                } else {
                    intakeSubsystem.off();
                }
            } else {
                // nothing
                intakeSubsystem.off();
            }

            if (gamepad1.left_trigger_pressed) {
                shooterSubsystem.setShoot();
            } else if (isMoving) {
                shooterSubsystem.setOff();
            } else {
                shooterSubsystem.setIdle(); // idle when not moving
            }

            shooterSubsystem.update(follower);
            turretSubsystem.update(follower);
            hoodSubsystem.update(follower);
            shooterSubsystem.update(follower);

            if (gamepad1.dpad_up) {
                turretSubsystem.smartServoController.setTargetRotation(0);
            }

            if (usePedroDrive) {
                if (isFollowingPath && !follower.isBusy()) {
                    isFollowingPath = false;
                    follower.startTeleOpDrive(false);
                }
                if (!follower.isBusy())
                    follower.setTeleOpDrive(-forward, right, -turn, isRedAlliance() ? Math.PI : 0);
            } else {
                driveSubsystem.driveFieldRelative(forward, right, turn, follower);
            }

            follower.update();

            telemetry.addData("isMoving", isMoving);
            telemetry.addData("Alliance", alliance.name());
            telemetry.addData("Robot X", follower.getPose().getX());
            telemetry.addData("Robot Y", follower.getPose().getY());
            telemetry.addData("Robot H", follower.getHeading());
            telemetry.addData("Turret rotation", turretSubsystem.getTurretAngle());
            telemetry.addData("Follower busy?", follower.isBusy());
            telemetry.addLine("---");
            telemetry.addData("looptime (ms)", timer.milliseconds());
            telemetry.update();
        }
    }
}
