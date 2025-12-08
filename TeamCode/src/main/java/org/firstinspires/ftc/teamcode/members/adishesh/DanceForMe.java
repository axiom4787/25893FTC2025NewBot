package org.firstinspires.ftc.teamcode.adikan;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Adi's auto dance")
public class DanceForMe extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;

    private int leftPos;
    private int rightPos;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        // forward 3, backward 3, clockwise 2, anticlockwise 2, left 3, right 3

        int count = 1;
        while(opModeIsActive() && count <= 3) {

                move(0.3, 0, 0);
                sleep(3000);
                move(-0.3, 0, 0);
                sleep(3000);
                move(0, 0, 0.3);
                sleep(3000);
                move(0, 0, -0.3);
                sleep(3000);
                move(0, 0.3, 0);
                sleep(3000);
                move(0, -0.3, 0);
                sleep(3000);
                ++count;

        }

    }

    private void move(double axial, double lateral, double yaw) {
            frontLeftDrive.setPower(axial+lateral+yaw);
            frontRightDrive.setPower(axial-lateral-yaw);
            rearLeftDrive.setPower(axial-lateral+yaw);
            rearRightDrive.setPower(axial+lateral-yaw);
    }


}

