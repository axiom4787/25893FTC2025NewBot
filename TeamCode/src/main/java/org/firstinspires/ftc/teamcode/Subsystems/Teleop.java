package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Boilerplate.LinearOpModeWithAlliance;
import org.firstinspires.ftc.teamcode.Boilerplate.Shared;
import org.firstinspires.ftc.teamcode.Hardware.CachingHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

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
        follower.setStartingPose(Shared.autoEndPose);

        CachingHardware.init(hardwareMap);
        hoodSubsystem = new HoodSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        limeLightSubsystem = new LimeLightSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        turretSubsystem = new TurretSubsystem();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        allHubs.forEach(hub -> hub.setConstant(0x8000FF));

        selectAlliance();
        Shared.setAlliance(alliance);

        ElapsedTime timer = new ElapsedTime();

        follower.startTeleOpDrive(false);

        while (opModeIsActive()) {
            timer.reset();

            allHubs.forEach(LynxModule::clearBulkCache);

            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            boolean isMoving = right != 0 || forward != 0 || turn != 0;

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
            } else if (gamepad1.left_bumper) {
                intakeSubsystem.reverse();
            } else {
                // nothing
                intakeSubsystem.off();
            }

            if (gamepad1.right_trigger_pressed) {
                shooterSubsystem.setShoot();
            } else if (isMoving) {
                shooterSubsystem.setOff();
            } else if (gamepad1.right_bumper) {
                shooterSubsystem.setReverse();
            } else {
                shooterSubsystem.setIdle();
            }

            shooterSubsystem.update(follower);
            turretSubsystem.update(follower);
            hoodSubsystem.update(follower);

            if (isFollowingPath && !follower.isBusy()) {
                isFollowingPath = false;
                follower.startTeleOpDrive(false);
            }
            if (!follower.isBusy())
                follower.setTeleOpDrive(-forward, right, -turn, false, isRedAlliance() ? Math.PI : 0);

            follower.update();

            double
                    x = follower.getPose().getX(),
                    y = follower.getPose().getY(),
                    h = Math.toDegrees(follower.getHeading());

            telemetry.addData("Alliance", alliance.name());
            telemetry.addData("Robot X | Y | H", "%4.2f | %4.2f | %4.2f", x, y, h);
            telemetry.addData("Turret angle", turretSubsystem.getCurrentAngle());
            telemetry.addData("Turret target", turretSubsystem.getTargetAngle());
            telemetry.addData("Turret raw power", turretSubsystem.turretController.getPower());
            telemetry.addData("Turret bad", turretSubsystem.turretController.bad);
            telemetry.addData("Shooter vel", shooterSubsystem.getVelocity());
            telemetry.addData("Shooter target", shooterSubsystem.getTargetVelocity());
            telemetry.addLine("---");
            telemetry.addData("looptime (ms)", timer.milliseconds());
            telemetry.update();
        }
    }
}
