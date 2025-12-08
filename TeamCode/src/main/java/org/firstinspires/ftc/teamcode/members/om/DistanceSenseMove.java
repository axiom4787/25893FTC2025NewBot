package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Om - Stop At Object")
public class DistanceSenseMove extends LinearOpMode {

    private DcMotor frontLeft, frontRight, rearLeft, rearRight;
    private DistanceSensor distance_sensor;

    private static final double POWER = 0.2;        // Slow and safe
    private static final double STOP_WHEN_CLOSER_THAN_CM = 45.0;  // Stop at ~45 cm
    private static final double MAX_TIME_SECONDS = 9.0;          // Safety timeout (~7–8 feet)

    @Override
    public void runOpMode() {

        // Map hardware
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        rearLeft   = hardwareMap.get(DcMotor.class, "rear_left_motor");
        rearRight  = hardwareMap.get(DcMotor.class, "rear_right_motor");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Correct motor directions (standard mecanum)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

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
            frontLeft.setPower(POWER);
            frontRight.setPower(POWER);
            rearLeft.setPower(POWER);
            rearRight.setPower(POWER);

            telemetry.update();
        }

        // STOP
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        telemetry.addData("Status", "Finished – Stopped safely");
        telemetry.update();
        sleep(500);
    }
}

