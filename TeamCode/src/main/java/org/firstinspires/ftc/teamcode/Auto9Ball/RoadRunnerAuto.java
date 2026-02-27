package org.firstinspires.ftc.teamcode.Auto9Ball;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

@Autonomous(name="Auto BLUE")
public abstract class RoadRunnerAuto extends LinearOpMode {
    DcMotor intakeMotor, shooterMotor, indexerMotor;
    DcMotorEx smartShooter;
    Config config = new Config();

    public enum Alliance { RED, BLUE }
    Alliance alliance;

    abstract Alliance getAlliance();

    // reverse if on red alliance
    public double i(double n) {
        return alliance == Alliance.RED ? -n : n;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        alliance = getAlliance();

        config.init(hardwareMap);

        intakeMotor = config.intake;
        shooterMotor = config.shooter;
        indexerMotor = config.indexer;
        smartShooter = (DcMotorEx) shooterMotor;

        smartShooter.setVelocityPIDFCoefficients(600, 0, 0, 14.6);

        // -53, -47
        // TODO: fix roadrunner
        Pose2d beginPose = new Pose2d(-54, i(-50), Math.toRadians(i(234)));

        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) throw new RuntimeException();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        VelConstraint velConstraint = (pose2dDual, posePath, v) -> 12.5;

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                    .stopAndAdd(new StartShooterAction())
                    .lineToY(i(-30))
                    .waitSeconds(1.0)
                    .stopAndAdd(new ShootAction())

                    .strafeToLinearHeading(new Vector2d(-12, i(-25)), Math.toRadians(i(270)))
                    .stopAndAdd(new StartIntakeAction())
                    .strafeToLinearHeading(new Vector2d(-12, i(-45)), Math.toRadians(i(270)), velConstraint)
                    .stopAndAdd(new StopIntakeAction())
                    .strafeToLinearHeading(new Vector2d(-43, i(-30)), Math.toRadians(i(234)))
                    .stopAndAdd(new ShootAction())

                    .strafeToLinearHeading(new Vector2d(12, i(-25)), Math.toRadians(i(270)))
                    .stopAndAdd(new StartIntakeAction())
                    .strafeToLinearHeading(new Vector2d(12, i(-45)), Math.toRadians(i(270)), velConstraint)
                    .stopAndAdd(new StopIntakeAction())
                    .strafeToLinearHeading(new Vector2d(-43, i(-30)), Math.toRadians(i(234)))
                    .stopAndAdd(new ShootAction())
                    
                    .strafeToLinearHeading(new Vector2d(-15, i(-40)), Math.toRadians(i(270)))

                    .build());
    }

    class StartIntakeAction implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(0.85);
            indexerMotor.setPower(-1.0);
        }
    }

    class StopIntakeAction implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(0.0);
            indexerMotor.setPower(0.0);
        }
    }

    class ShootAction implements InstantFunction {
        @Override
        public void run() {
            sleep(1000);
            intakeMotor.setPower(1.0);
            indexerMotor.setPower(0.4);
            sleep(4000);
            intakeMotor.setPower(0.0);
            indexerMotor.setPower(0.0);
        }
    }

    class StartShooterAction implements InstantFunction {
        @Override
        public void run() {
            smartShooter.setVelocity(1450);
        }
    }
}
