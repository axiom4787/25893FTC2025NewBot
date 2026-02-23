package org.firstinspires.ftc.teamcode.Auto9Ball;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
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
import org.jetbrains.annotations.NotNull;

@Autonomous(name="Auto RED")
public class RoadRunnerAutoRED extends LinearOpMode {
    DcMotor intakeMotor, shooterMotor, indexerMotor;
    DcMotorEx smartShooter;
    Config config = new Config();

    enum Alliance { RED, BLUE, }
    Alliance alliance = Alliance.RED;

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

        VelConstraint velConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
                return 10;
            }
        };

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
            sleep(4000);
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
