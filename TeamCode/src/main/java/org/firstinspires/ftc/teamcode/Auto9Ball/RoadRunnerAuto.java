package org.firstinspires.ftc.teamcode.Auto9Ball;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

//@Autonomous(name="Auto")
public class RoadRunnerAuto extends LinearOpMode {
    DcMotor intakeMotor, shooterMotor, indexerMotor;
    DcMotorEx smartShooter;
    Config config = new Config();

    enum Alliance { RED, BLUE, }
    Alliance alliance = Alliance.BLUE;

    // reverse if on red alliance
    public double i(double n) {
        return alliance == Alliance.RED ? -n : n;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        config.init(hardwareMap);

        intakeMotor = config.intake;
        shooterMotor = config.shooter;
        indexerMotor = config.indexer;
        smartShooter = (DcMotorEx) shooterMotor;

        double P = 600;
        double I = 0.0;
        double D = 0;
        double F = 14.6;
        smartShooter.setVelocityPIDFCoefficients(P, I, D, F);

        Pose2d beginPose = new Pose2d(-56, i(-47), Math.toRadians(i(234)));

        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) throw new RuntimeException();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                    .stopAndAdd(new StartShooterAction())
                    .lineToY(i(-30))
                    .waitSeconds(1.0)
                    .stopAndAdd(new ShootAction())

                    .strafeToLinearHeading(new Vector2d(-12, i(-25)), Math.toRadians(i(270)))
                    .stopAndAdd(new StartIntakeAction())
                    .strafeToLinearHeading(new Vector2d(-12, i(-45)), Math.toRadians(i(270)))
                    .splineToLinearHeading(new Pose2d(-43, i(-30), Math.toRadians(i(234))), Math.toRadians(i(180)))
                    .afterDisp(1.0, new StopIntakeAction())
                    .stopAndAdd(new ShootAction())

                    .strafeToLinearHeading(new Vector2d(12, i(-25)), Math.toRadians(i(270)))
                    .stopAndAdd(new StartIntakeAction())
                    .strafeToLinearHeading(new Vector2d(12, i(-45)), Math.toRadians(i(270)))
                    .splineToLinearHeading(new Pose2d(-43, i(-30), Math.toRadians(i(234))), Math.toRadians(i(180)))
                    .afterDisp(1.0, new StopIntakeAction())
                    .stopAndAdd(new ShootAction())

                    .build());
    }

    class StartIntakeAction implements Action {
        @Override
        public boolean run(TelemetryPacket p) {
            intakeMotor.setPower(0.7);
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
            sleep(1000);
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
            smartShooter.setVelocity(1450);

            return false;
        }
    }
}
