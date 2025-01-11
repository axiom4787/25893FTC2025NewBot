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
public class BlueAutonomous extends LinearOpMode {

    Servo intakeTilt = null;
    Servo intake = null;
    Servo xfer = null;
    Servo leftIntake = null;
    Servo rightIntake = null;

    DcMotor rightSlideMotor = null;
    DcMotor leftSlideMotor = null;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        intake = hardwareMap.get(Servo.class, "intake");
        xfer = hardwareMap.get(Servo.class, "xfer");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");

        leftSlideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set servo positions before the robot starts moving
        intakeTilt.setPosition(0.3);
        intake.setPosition(0); // intake

        xfer.setPosition(0);
        rightIntake.setPosition(0.25); // top intake right
        leftIntake.setPosition(0.75); // top intake left

        //

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(20)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(150)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(50)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(5)
                .build();

        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj1);
        sleep(1000);
        leftSlideMotor.setPower(-0.6);
        rightSlideMotor.setPower(0.6);
        sleep(2000);
        rightIntake.setPosition(0.65); // top intake right
        leftIntake.setPosition(0.35); // top intake left
        sleep(1000);
        xfer.setPosition(0.15);
        sleep(1000);
        rightIntake.setPosition(0.06);
        leftIntake.setPosition(0.94);
        sleep(1000);
        leftSlideMotor.setPower(1);
        rightSlideMotor.setPower(-1); // put it down
        sleep(1500);
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
    }
}
