package org.firstinspires.ftc.teamcode.members.ishani.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "IS - Sensor")
public class IshaniSensor extends LinearOpMode {

    // Motors
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // REV Robotics 2m Distance Sensor (infrared laser)
     DistanceSensor distanceSensor;

    private static final double TARGET_DISTANCE_INCHES = 120;        // 10 feet
    private static final double DRIVE_POWER = 0.2;              // Drive Power 20%
    private static final double STOP_IF_CLOSER_THAN_CM = 30;// ~12 inches


    @Override
    public void runOpMode() {

        // === HARDWARE MAP ===
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right_motor");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // Reverse left side so robot goes straight
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData(">", "Robot Ready – REV 2m Sensor Active!");
        telemetry.update();

        waitForStart();
        resetRuntime(); // start timer for rough distance estimate

        while (opModeIsActive()) {

            double traveledCm = getRuntime() * DRIVE_POWER;

            // Get distance from the REV sensor
            double objectDistance = distanceSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("Traveled", "%.1f / 305 cm", traveledCm);
            telemetry.addData("Object Ahead", "%.1f cm", objectDistance);
            telemetry.update();

            // SUCCESS: Reached 10 feet
            if (traveledCm >= TARGET_DISTANCE_INCHES) {
                stopRobot();
                telemetry.addData("DONE", "Reached 10 feet!");
                telemetry.update();
                break;
            }

            //  STOP: Something is too close!
            if (objectDistance < STOP_IF_CLOSER_THAN_CM && objectDistance < 8000) {
                stopRobot();
                telemetry.addData("STOPPED", "Object detected at %.1f cm!", objectDistance);
                telemetry.update();
                sleep(2000); // pause 2 sec so you can read the message
                break;
            }

            // Keep driving forward
            driveForward(DRIVE_POWER);
        }

        // Always make sure motors are off when done
        stopRobot();
    }

    // Helper methods – makes code cleaner
    private void driveForward(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }
    private void stopRobot() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}