package org.firstinspires.ftc.teamcode.TestOpModes;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

@Autonomous(name="test auto not for comp")
public class TestAutoLocation extends LinearOpMode {
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

                        .strafeToLinearHeading(new Vector2d(0, 0), java.lang.Math.toRadians(0))

                        .build());
        }
    }