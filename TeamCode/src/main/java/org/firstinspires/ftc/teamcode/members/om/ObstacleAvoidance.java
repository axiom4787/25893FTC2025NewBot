package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Om - Obstacle Avoidance")
public class ObstacleAvoidance extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private DistanceSensor distance_sensor;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    static final double COUNTS_PER_90_DEGREE = 1050;

    static final double OBSTACLE_THRESHOLD_INCHES = 16.0;
    static final double DETOUR_SIDE_INCHES = 18.0;
    static final double TOTAL_FORWARD_INCHES = 68.0; // 5 feet

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");

        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double distanceTraveled = 0.0;

        while (opModeIsActive() && distanceTraveled < TOTAL_FORWARD_INCHES) {

            double distance = distance_sensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Distance", "%.1f in", distance);
            telemetry.addData("Progress", "%.1f / 84.0 in", distanceTraveled);
            telemetry.update();

            if (distance < OBSTACLE_THRESHOLD_INCHES && distance > 0) {

                stopMotors();

                turnRight90();
                driveStraight(DETOUR_SIDE_INCHES);
                turnLeft90();
                driveStraight(DETOUR_SIDE_INCHES + 15.0); // pass the object
                turnLeft90();
                driveStraight(DETOUR_SIDE_INCHES - 6.0);
                turnRight90();

                distanceTraveled += DETOUR_SIDE_INCHES;

                if (distanceTraveled > TOTAL_FORWARD_INCHES) {
                    distanceTraveled = TOTAL_FORWARD_INCHES;
                    stopMotors();
                }
            } else {

                double remaining = TOTAL_FORWARD_INCHES - distanceTraveled;
                double chunk = Math.min(10.0, remaining);

                driveStraight(chunk);
                distanceTraveled += chunk;
            }
        }

        stopMotors();
        telemetry.addData("Path", "Complete - 7 feet traveled");
        telemetry.update();
    }


    public void driveStraight(double inches) {
        int target = (int)(inches * COUNTS_PER_INCH);

        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + target);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + target);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + target);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + target);

        setRunToPosition();
        setPower(DRIVE_SPEED);

        while (opModeIsActive() && motorsBusy()) {
            telemetry.addData("Driving", "%.1f in", inches);
            telemetry.update();
        }

        stopMotors();
        resetEncoders();
    }

    public void turnRight90() {
        int target = (int)COUNTS_PER_90_DEGREE;

        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() + target);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() - target);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + target);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() - target);

        setRunToPosition();
        setPower(TURN_SPEED);

        while (opModeIsActive() && motorsBusy()) {}

        stopMotors();
        resetEncoders();
    }

    public void turnLeft90() {
        int target = (int)COUNTS_PER_90_DEGREE;

        frontLeftDrive.setTargetPosition(frontLeftDrive.getCurrentPosition() - target);
        frontRightDrive.setTargetPosition(frontRightDrive.getCurrentPosition() + target);
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() - target);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + target);

        setRunToPosition();
        setPower(TURN_SPEED);

        while (opModeIsActive() && motorsBusy()) {}

        stopMotors();
        resetEncoders();
    }

    public void setPower(double power) {
        frontLeftDrive.setPower(Math.abs(power));
        frontRightDrive.setPower(Math.abs(power));
        backLeftDrive.setPower(Math.abs(power));
        backRightDrive.setPower(Math.abs(power));
    }

    public void setRunToPosition() {
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean motorsBusy() {
        return frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                backLeftDrive.isBusy() || backRightDrive.isBusy();
    }

    public void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void resetEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}