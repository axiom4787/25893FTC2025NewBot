package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

 
public class OurAuto extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private float positionPerDegree = 1 / (270 / 2);
    private Servo flap;
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flap = hardwareMap.get(Servo.class, "servo");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        flywheel.setVelocity(1300);
        sleep(5000);
        flap.setPosition(90 * positionPerDegree);
        sleep(5000);
    }
}
