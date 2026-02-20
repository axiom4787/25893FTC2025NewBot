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
    DcMotor intakeMotor, shooterMotor, indexerMotor;
    Config config = new Config();

    @Override
    public void runOpMode() throws InterruptedException {
        config.init(hardwareMap);

        intakeMotor = config.intake;
        shooterMotor = config.shooter;
        indexerMotor = config.indexer;

        Pose2d beginPose = new Pose2d(-56, -47, Math.toRadians(234));

        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) throw new RuntimeException();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                    .waitSeconds(0.5)
                    .stopAndAdd(new StartShooterAction())
                    .lineToY(-30)
                    .waitSeconds(1.0)
                    .stopAndAdd(new ShootAction())
                    .waitSeconds(0.5)

                    .strafeToLinearHeading(new Vector2d(-12, -25), Math.toRadians(270))
                    .stopAndAdd(new StartIntakeAction())
                    .strafeToLinearHeading(new Vector2d(-12, -45), Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(-43, -30, Math.toRadians(234)), Math.toRadians(180))
                    .afterDisp(1.0, new StopIntakeAction())
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
            intakeMotor.setPower(0.8);
//            indexerMotor.setPower(-0.2);

            return false;
        }
    }

    class StopIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            intakeMotor.setPower(0.0);
//            indexerMotor.setPower(0.0);

            return false;
        }
    }

    class ShootAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            intakeMotor.setPower(1.0);
            indexerMotor.setPower(0.3);
            sleep(5000);
            intakeMotor.setPower(0.0);
            indexerMotor.setPower(0.0);

            return false;
        }
    }

    class StartShooterAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            shooterMotor.setPower(0.7);

            return false;
        }
    }
}
