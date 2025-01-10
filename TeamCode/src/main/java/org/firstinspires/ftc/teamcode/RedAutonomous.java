package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class RedAutonomous extends LinearOpMode {

    Servo intakeTilt = null;
    Servo intake = null;
    Servo xfer = null;
    Servo leftIntake = null;
    Servo rightIntake = null;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        intake = hardwareMap.get(Servo.class, "intake");
        xfer = hardwareMap.get(Servo.class, "xfer");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set servo positions before the robot starts moving
        intakeTilt.setPosition(0.3);
        intake.setPosition(0); // intake

        xfer.setPosition(0);
        rightIntake.setPosition(0.25); // top intake right
        leftIntake.setPosition(0.75); // top intake left


        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(24)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(20)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(50)
                .build();

        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectory(traj1);
        rightIntake.setPosition(0.95); // top intake right
        leftIntake.setPosition(0.05); // top intake left
        sleep(1000);
        drive.followTrajectory(traj2);
        sleep(1000);
        drive.followTrajectory(traj3);

    }
}
