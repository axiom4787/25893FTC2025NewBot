package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Auto_Amulya {


    @Autonomous(name="AutoMode_Hinaa", group="Robot")
    public class AutoMode_Hinaa extends LinearOpMode {

        private DcMotor frontleft, frontright, backleft, backright;
        private ElapsedTime runtime = new ElapsedTime();

        static final double COUNTS_PER_MOTOR_REV = 383.6;
        static final double DRIVE_GEAR_REDUCTION = 1.0;
        static final double WHEEL_DIAMETER_INCHES = 4.0;
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double DRIVE_SPEED = 0.6;

        @Override
        public void runOpMode() {

            frontleft = hardwareMap.get(DcMotor.class, "frontleft");
            frontright = hardwareMap.get(DcMotor.class, "frontright");
            backleft = hardwareMap.get(DcMotor.class, "backleft");
            backright = hardwareMap.get(DcMotor.class, "backright");

            frontleft.setDirection(DcMotor.Direction.REVERSE);
            backleft.setDirection(DcMotor.Direction.REVERSE);
            frontright.setDirection(DcMotor.Direction.FORWARD);
            backright.setDirection(DcMotor.Direction.FORWARD);

            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            waitForStart();

            // Forward 8 inches
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + (int)(12 * COUNTS_PER_INCH));
            frontright.setTargetPosition(frontright.getCurrentPosition() + (int)(12 * COUNTS_PER_INCH));
            backleft.setTargetPosition(backleft.getCurrentPosition() + (int)(12 * COUNTS_PER_INCH));
            backright.setTargetPosition(backright.getCurrentPosition() + (int)(12 * COUNTS_PER_INCH));

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleft.setPower(DRIVE_SPEED);
            frontright.setPower(DRIVE_SPEED);
            backleft.setPower(DRIVE_SPEED);
            backright.setPower(DRIVE_SPEED);

            while (opModeIsActive() && frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {}

            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

            sleep(250);

            // Backward 8 inches
            frontleft.setTargetPosition(frontleft.getCurrentPosition() - (int)(12 * COUNTS_PER_INCH));
            frontright.setTargetPosition(frontright.getCurrentPosition() - (int)(12 * COUNTS_PER_INCH));
            backleft.setTargetPosition(backleft.getCurrentPosition() - (int)(12 * COUNTS_PER_INCH));
            backright.setTargetPosition(backright.getCurrentPosition() - (int)(12 * COUNTS_PER_INCH));

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleft.setPower(DRIVE_SPEED);
            frontright.setPower(DRIVE_SPEED);
            backleft.setPower(DRIVE_SPEED);
            backright.setPower(DRIVE_SPEED);

            while (opModeIsActive() && frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {}

            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);

            sleep(250);

            // Strafe right 8 inches (mecanum)
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + (int)(12 * COUNTS_PER_INCH));
            frontright.setTargetPosition(frontright.getCurrentPosition() - (int)(12 * COUNTS_PER_INCH));
            backleft.setTargetPosition(backleft.getCurrentPosition() - (int)(12 * COUNTS_PER_INCH));
            backright.setTargetPosition(backright.getCurrentPosition() + (int)(12 * COUNTS_PER_INCH));

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleft.setPower(DRIVE_SPEED);
            frontright.setPower(DRIVE_SPEED);
            backleft.setPower(DRIVE_SPEED);
            backright.setPower(DRIVE_SPEED);

            while (opModeIsActive() && frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {}

            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
    }

}
