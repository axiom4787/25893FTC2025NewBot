package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

@Autonomous(name="Auto")
public final class RoadRunnerAuto extends LinearOpMode {
    DcMotor intakeMotor, shooterMotor;
    Config config = new Config();

    @Override
    public void runOpMode() throws InterruptedException {
        config.init(hardwareMap);

        intakeMotor = config.intake;
        shooterMotor = config.shooter;

        Pose2d beginPose = new Pose2d(-55, -44, Math.toRadians(234));

        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) throw new RuntimeException();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                    .waitSeconds(0.5)
                    .lineToY(-30)
                    .stopAndAdd(new ShootAction())
                    .waitSeconds(0.5)

                    .strafeToLinearHeading(new Vector2d(-12, -25), Math.toRadians(270))
                    .stopAndAdd(new StartIntakeAction())
                    .strafeToLinearHeading(new Vector2d(-12, -45), Math.toRadians(270))
                    .stopAndAdd(new StopIntakeAction())
                    .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(234)), Math.toRadians(180))
                    .stopAndAdd(new ShootAction())
                    .waitSeconds(0.5)

//                    .strafeToLinearHeading(new Vector2d(12, -25), Math.toRadians(270))
//                    .strafeToLinearHeading(new Vector2d(12, -45), Math.toRadians(270))
//                    .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(234)), Math.toRadians(180))
//                    .afterDisp(0.0, new IntakeAction())
//                    .stopAndAdd(new ShootAction())
//                    .waitSeconds(0.5)
                    .build());
    }

    class StartIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            intakeMotor.setPower(0.6);
            shooterMotor.setPower(-0.4);

            return false;
        }
    }

    class StopIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            intakeMotor.setPower(0.0);
            shooterMotor.setPower(0.0);

            return false;
        }
    }

    class ShootAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            shooterMotor.setPower(0.6);
            sleep(2500);
            intakeMotor.setPower(-0.4);
            sleep(200);
            intakeMotor.setPower(1.0);
            sleep(5000);
            shooterMotor.setPower(0.0);
            intakeMotor.setPower(0.0);

            return false;
        }
    }
}
