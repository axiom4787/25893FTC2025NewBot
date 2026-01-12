package org.firstinspires.ftc.teamcode.members.bala;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class RotateRobot_Bala extends LinearOpMode {

    // todo: write your code here

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;


    private void RobotDance(int times) {
        for (int i = 0; i < times; i++){

        move(1, 0, 0, 0.2);
        sleep(3000);
        move(-1, 0, 0, 0.2);
        sleep(3000);
        move(0, 1, 0, 0.2);
        sleep(3000);
        move(0, -1, 0, 0.2);
        sleep(3000);
        move(0, 0, 1, 0.2);
        sleep(2000);
        move(0, 0, -1,  0.2);
        sleep(2000);
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive()) {
            RobotDance(3);
        }
    }



    private void move(int axial, int lateral, int yaw, double velocity) {

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;
        if(frontLeftPower > 1) {
        frontLeftPower = 1;
        }
        if(frontRightPower > 1) {
            frontRightPower = 1;
        }
        if(backLeftPower > 1) {
            backRightPower = 1;
        }
        if(backRightPower > 1) {
            backRightPower = 1;
        }




        if(frontLeftPower < -1) {
            frontLeftPower = -1;
        }
        if(frontRightPower < -1) {
            frontRightPower = -1;
        }
        if(backLeftPower < -1) {
            backRightPower = -1;
        }
        if(backRightPower < -1) {
            backRightPower = -1;
        }

        if(velocity > 1)
            velocity = 1;

        if(velocity < 0)
            velocity = 0;


        frontLeftDrive.setPower(frontLeftPower * velocity);
        frontRightDrive.setPower(frontRightPower * velocity);
        backLeftDrive.setPower(backLeftPower * velocity);
        backRightDrive.setPower(backRightPower * velocity);


    }

}