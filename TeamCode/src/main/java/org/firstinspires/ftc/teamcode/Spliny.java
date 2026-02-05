package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="Spline test")
public final class Spliny extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-55, -44, Math.toRadians(234));

        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) throw new RuntimeException();

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                    .waitSeconds(0.5)
                    .lineToY(-30)
                    .waitSeconds(0.5)

                    .strafeToLinearHeading(new Vector2d(-12, -25), Math.toRadians(270))
                    .strafeToLinearHeading(new Vector2d(-12, -45), Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(234)), Math.toRadians(180))
                    .waitSeconds(0.5)

                    .strafeToLinearHeading(new Vector2d(12, -25), Math.toRadians(270))
                    .strafeToLinearHeading(new Vector2d(12, -45), Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(234)), Math.toRadians(180))
                    .waitSeconds(0.5)
                    .build());
    }
}
