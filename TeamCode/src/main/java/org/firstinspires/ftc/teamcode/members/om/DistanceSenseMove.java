package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Om - Stop At Object")
public class DistanceSenseMove extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive;
    private DistanceSensor distance_sensor;

    private static final double POWER = 0.2;        // Slow and safe
    private static final double STOP_WHEN_CLOSER_THAN_CM = 45.0;  // Stop at ~45 cm
    private static final double MAX_TIME_SECONDS = 9.0;          // Safety timeout (~7–8 feet)

    @Override
    public void runOpMode() {

        // Map hardware
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_motor");
        rearRightDrive  = hardwareMap.get(DcMotor.class, "back_right_motor");

        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Correct motor directions (standard mecanum)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Ready – Press Play");
        telemetry.update();

        waitForStart();
        resetRuntime(); // Starts timer at 0

        while (opModeIsActive()) {

            double distance = 999;  // default = far away

            distance = distance_sensor.getDistance(DistanceUnit.CM);

            telemetry.addData("Distance (cm)", "%.1f", distance);
            telemetry.addData("Time (sec)", "%.1f", getRuntime());

            // Stop conditions
            if (distance < STOP_WHEN_CLOSER_THAN_CM && distance > 8) {
                telemetry.addData("STOP", "Object detected!");
                break;
            }

            if (getRuntime() > MAX_TIME_SECONDS) {
                telemetry.addData("STOP", "Time limit reached");
                break;
            }

            // Keep driving forward
            frontLeftDrive.setPower(POWER);
            frontRightDrive.setPower(POWER);
            rearLeftDrive.setPower(POWER);
            rearRightDrive.setPower(POWER);

            telemetry.update();
        }

        // STOP
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        telemetry.addData("Status", "Finished – Stopped safely");
        telemetry.update();
        sleep(500);
    }
}

