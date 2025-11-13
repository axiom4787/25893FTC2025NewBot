package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Just Shoot")
public class JustShoot extends LinearOpMode{
    DcMotorEx motorShooterLeft;
    DcMotorEx motorShooterRight;
    DcMotor motorTransfer;

    double timer;

    double shooterLimit;
    double shooterRightVelocity;

    int where;

    @Override
    public void runOpMode(){ //throws InterruptedException
        motorShooterLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "motorShooterLeft");
        motorShooterRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "motorShooterRight");
        motorTransfer = (DcMotorEx)hardwareMap.get(DcMotor.class, "motorTransfer");

        shooterLimit = -2700;

        timer = 0;

        waitForStart();

        where = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Where", where);
        telemetry.addData("Left Velocity", shooterRightVelocity);
        telemetry.update();
        shooterRightVelocity = ((motorShooterRight.getVelocity()/28)*60);


        if(opModeIsActive()) {
            if (shooterRightVelocity<2700 && timer <2000){
                motorShooterLeft.setPower(1.0);
                motorShooterRight.setPower(-1.0);
                where = 1;
            }
            else if(shooterRightVelocity>2700 && timer <2000) {
                motorShooterLeft.setPower(1.0);
                motorShooterRight.setPower(-1.0);
                motorTransfer.setPower(-0.6);
                timer++;
                where = 2;
            }
            else{
                motorShooterLeft.setPower(0.0);
                motorShooterRight.setPower(0.0);
                motorTransfer.setPower(0.0);
                where = 3;
            }


            sleep(200000);

        }
    }
}