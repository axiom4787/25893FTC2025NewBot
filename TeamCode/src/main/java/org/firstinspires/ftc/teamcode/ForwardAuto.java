package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class ForwardAuto extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        waitForStart();
        leftDrive.setPower(1);
        rightDrive.setPower(0.65);
        sleep(700);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
