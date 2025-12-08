package org.firstinspires.ftc.teamcode.FRLib.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FRLib.hardware.Distance2mW;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.hardware.MotorW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;

/**
 * MecanumDrive subsystem for FTC robots.
 * Handles mecanum wheel movement, distance-based driving, strafing, and PID turning.
 */
public class MecanumDrive {

    private final LinearOpMode opMode;
    private final Logger logger;
    private final ElapsedTime runtime = new ElapsedTime();

    // Wheel motors
    public final MotorW frontLeft, frontRight, rearLeft, rearRight;
    public final MotorW[] motors;

    // PID controller for turning and drive
    public final PIDController turnPid;
    public final PIDController drivePid;

    // Encoder and wheel constants
    private static final float COUNTS_PER_REV = 537.7f;
    private static final float GEAR_RATIO = 1.0f;
    private static final float WHEEL_DIAMETER_MM = 104.0f;
    private static final float COUNTS_PER_MM =
            (COUNTS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_MM * (float)Math.PI);
    private static final float COUNTS_PER_INCH = COUNTS_PER_MM * 25.4f;

    private final double OVERSHOOT_PER_INCH;

    /** Cardinal directions for movement convenience */
    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT
    }

    /** different robots have different characteristics */
    public enum RobotName {
        KEVIN,
        BOB
    }

    /*
    Init/Constructor
    */

    public MecanumDrive(LinearOpMode opMode, Logger logger, RobotName robotName) {
        this.opMode = opMode;
        this.logger = logger;

        switch (robotName) {
            case KEVIN:
                OVERSHOOT_PER_INCH = 1.036458333333333333;
                break;
            case BOB:
                OVERSHOOT_PER_INCH = 1.036458333333333333;
                break;
            default:
                OVERSHOOT_PER_INCH = 1;
                break;
        }

        // Initialize all 4 drive motors
        frontLeft = createMotor("front_left_motor", DcMotor.Direction.REVERSE);
        frontRight = createMotor("front_right_motor", DcMotor.Direction.FORWARD);
        rearLeft = createMotor("back_left_motor", DcMotor.Direction.REVERSE);
        rearRight = createMotor("back_right_motor", DcMotor.Direction.FORWARD);

        motors = new MotorW[]{frontLeft, frontRight, rearLeft, rearRight};

        // init in constructor since they need logger
        turnPid = new PIDController(0.2f, 0, 0.02f, logger);
        drivePid = new PIDController(0.1f, 0.05f, 0.03f, logger);

        stopAndResetEncoders();
    }

    /** Helper for creating a motor */
    private MotorW createMotor(String name, DcMotor.Direction dir) {
        return new MotorW(opMode, name, dir);
    }


    /*
     * Encoder & Motor Utility Methods
     */

    public int averageEncoderValues() {
        int sum = 0;
        for (MotorW motor : motors) {
            sum += motor.getPosition();
        }
        return sum / motors.length;
    }

    /** Reset all motors' encoders */
    public void stopAndResetEncoders() {
        for (MotorW m : motors) m.resetEncoder();
    }

    /** set all drive motor powers to input */
    public void setMotorPowers(double power) {
        for (MotorW m : motors) m.setPower(power);
    }

    public void setMotorModes(DcMotor.RunMode runMode) {
        for (MotorW motor : motors) {
            motor.setMode(runMode);
        }
    }
    
    public void setMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zpb) {
        for (MotorW motor : motors) motor.setZeroPowerBehavior(zpb);
    }

    /** Stop all motors immediately */
    public void stop() {
        setMotorPowers(0);
    }

    public void brake(long holdTimeMs) {
        // ROBOT SHOULD ALREADY BE IN BRAKE ZERO-POWER MODE
        for (MotorW motor : motors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Read target hold positions (encoder ticks)
        int flTarget = frontLeft.getPosition();
        int frTarget = frontRight.getPosition();
        int rlTarget = rearLeft.getPosition();
        int rrTarget = rearRight.getPosition();

        double kP = 0.01;       // proportional gain for holding
        double maxHoldPower = 0.3;   // limit to protect motors

        ElapsedTime timer = new ElapsedTime();

        // Run lightweight PID loop
        while (opMode.opModeIsActive() && timer.milliseconds() < holdTimeMs) {

            // Compute position errors
            double flError = flTarget - frontLeft.getPosition();
            double frError = frTarget - frontRight.getPosition();
            double rlError = rlTarget - rearLeft.getPosition();
            double rrError = rrTarget - rearRight.getPosition();

            // Compute hold power based on P only (I and D not needed)
            double flPower = Range.clip(flError * kP, -maxHoldPower, maxHoldPower);
            double frPower = Range.clip(frError * kP, -maxHoldPower, maxHoldPower);
            double rlPower = Range.clip(rlError * kP, -maxHoldPower, maxHoldPower);
            double rrPower = Range.clip(rrError * kP, -maxHoldPower, maxHoldPower);

            // Apply holding power
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            rearLeft.setPower(rlPower);
            rearRight.setPower(rrPower);

            logger.logData(Logger.LoggerMode.DETAILED, "Braking/Holding", holdTimeMs);
        }

        // Stop applying hold power after time expires
        stop();
    }


    /*
     * Robot Centric Driving
     */

    /**
     * Drive robot with vector components
     * @param axial forward/backward speed
     * @param lateral left/right speed
     * @param yaw rotation speed
     * @param power overall power (0-1)
     */
    public void driveVector(double axial, double lateral, double yaw, double power) {
        // Calculate raw motor powers
        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double rl = axial - lateral + yaw;
        double rr = axial + lateral - yaw;

        // Normalize powers so no value exceeds 1
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(rl), Math.abs(rr)))));
        // scales them down
        fl /= max;
        fr /= max;
        rl /= max;
        rr /= max;

        // Set motors to run without encoders for smooth control
        for (MotorW m : motors) m.runWithoutEncoder();

        // Apply power
        frontLeft.setPower(fl * power);
        frontRight.setPower(fr * power);
        rearLeft.setPower(rl * power);
        rearRight.setPower(rr * power);
    }


    /*
     * Distance-Based Movement
     */

    /**
     * Move robot in a cardinal direction for a specific distance
     * @param dir direction to move
     * @param inches distance to travel
     * @param power speed (0-1)
     * @param timeout maximum time to attempt movement
     * @param wait whether to wait for completion before returning
     */
    public void driveDistance(Direction dir, float inches, float power, float timeout, boolean wait) {
        switch (dir) {
            case FORWARD:
                driveDistanceTank(inches, inches, power, timeout, wait);
                break;
            case BACKWARD:
                driveDistanceTank(-inches, -inches, power, timeout, wait);
                break;
            case RIGHT:
                strafe(inches, power, timeout, wait);
                break;
            case LEFT:
                strafe(-inches, power, timeout, wait);
                break;
        }
    }

    /** Drive left and right sides independently (tank style) */
    public void driveDistanceTank(float leftInches, float rightInches, float power, float timeout, boolean wait) {
        moveMotorsDistance(new MotorW[]{frontLeft, rearLeft}, leftInches, power, timeout);
        moveMotorsDistance(new MotorW[]{frontRight, rearRight}, rightInches, power, timeout);
        if (wait) waitUntilDone(timeout);
    }

    /** Strafe robot sideways */
    private void strafe(float inches, float power, float timeout, boolean wait) {
        moveMotorsDistance(new MotorW[]{frontLeft, rearRight}, inches, power, timeout);
        moveMotorsDistance(new MotorW[]{frontRight, rearLeft}, -inches, power, timeout);
        if (wait) waitUntilDone(timeout);
    }

    /**
     * Move specified motors a certain distance
     * @param motors array of motors to move
     * @param inches distance in inches
     * @param power motor power (0-1)
     * @param timeout max time to attempt movement
     */
    private void moveMotorsDistance(@NonNull MotorW[] motors, float inches, float power, float timeout) {
        if (!opMode.opModeIsActive()) return;

        int targetCounts = (int) (inches * COUNTS_PER_INCH);
        for (MotorW m : motors) {
            m.runUsingEncoder();
            m.runToPosition(m.getPosition() + targetCounts);
            m.setPower(power);
        }
    }

    /** Wait until all motors stop moving or timeout */
    private void waitUntilDone(float timeout) {
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < timeout) {
            boolean anyBusy = false;
            for (MotorW m : motors) {
                if (m.isBusy()) {
                    anyBusy = true;
                    break;
                }
            }
            if (!anyBusy) break;
        }
        stop();
    }


    /**
     * Drive straight at a specified heading for a given distance using proportional control.
     * @param imu the robot IMU for heading feedback
     * @param angleDegrees target heading in degrees (0 = forward)
     * @param inches distance to travel
     * @param power base motor power (0-1)
     * @param timeout max time to attempt movement
     */
    public void driveStraight(@NonNull IMUW imu, float angleDegrees, float inches, float power, float timeout) {
        double kP = 0.02; // proportional gain
        double startYaw = imu.getYaw();
        runtime.reset();

        int targetCounts = (int)(Math.abs(inches) * COUNTS_PER_INCH);
        double direction = Math.signum(inches); // 1 for forward, -1 for backward

        // Reset encoders, run to position
        for (MotorW m : motors) {
            m.resetEncoder();
            m.runUsingEncoder();
            m.runToPosition(targetCounts);
        }

        while (opMode.opModeIsActive() && runtime.seconds() < timeout) {
            // check if any motor isn't finished
            boolean done = true;
            for (MotorW m : motors) {
                if (Math.abs(m.getPosition()) < targetCounts) {
                    done = false;
                    break;
                }
            }

            // if all motors are done moving to position, exit
            if (done) break;

            double currentYaw = imu.getYaw() - startYaw;
            double error = angleDegrees - currentYaw;

            error = AngleUnit.normalizeDegrees(error);

            double correction = kP * error;

            frontLeft.setPower(power + correction);
            rearLeft.setPower(power + correction);

            frontRight.setPower(power - correction);
            rearRight.setPower(power - correction);
        }

        brake(500);
    }

    public void driveStraightUnlessSeeInches(@NonNull IMUW imu, float angleDegrees, float inches, float power, float timeout, Distance2mW sensor, double tooCloseInches) {
        double kP = 0.02; // proportional gain
        double startYaw = imu.getYaw();
        runtime.reset();

        int targetCounts = (int)(Math.abs(inches) * COUNTS_PER_INCH);
        double direction = Math.signum(inches); // 1 for forward, -1 for backward

        // Reset encoders, run to position
        for (MotorW m : motors) {
            m.resetEncoder();
            m.runUsingEncoder();
            m.runToPosition(targetCounts);
        }

        while (opMode.opModeIsActive() && sensor.getDistance(DistanceUnit.INCH) < tooCloseInches && runtime.seconds() < timeout) {
            // check if any motor isn't finished
            boolean done = true;
            for (MotorW m : motors) {
                if (Math.abs(m.getPosition()) < targetCounts) {
                    done = false;
                    break;
                }
            }

            // if all motors are done moving to position, exit
            if (done) break;

            double currentYaw = imu.getYaw() - startYaw;
            double error = angleDegrees - currentYaw;

            error = AngleUnit.normalizeDegrees(error);

            double correction = kP * error;

            frontLeft.setPower(power + correction);
            rearLeft.setPower(power + correction);

            frontRight.setPower(power - correction);
            rearRight.setPower(power - correction);
        }

        brake(500);
    }


    /*
     * PID
     */

    /**
     * Turn robot to a specific heading using IMU and PID
     * @param imu the robot IMU
     * @param targetAngle desired yaw angle
     * @param power multiplied by the correction
     * @param toleranceDegrees how close to get to targetAngle before exiting (negative to hold forever)
     */
    public void turnDegreesPID(@NonNull IMUW imu, double targetAngle, double power, double toleranceDegrees) {
        turnPid.setOutputLimits(-1, 1); // limit motor output
        double startYaw = imu.getYaw();

        while (opMode.opModeIsActive()) {
            // this error is only used to know when to stop
            double current = imu.getYaw() - startYaw; // getYaw only returns from -180 to +180
            double error = targetAngle - current;

            // eg. target = 720, error will always be 720 -/+ 180, which will never be within tolerance
            error = AngleUnit.normalizeDegrees(error);

            // Stop if within tolerance
            if (Math.abs(error) <= toleranceDegrees) break;

            double correction = turnPid.getOutput(targetAngle, current);
            driveVector(0, 0, correction, power);

            logger.update();
        }

        stop();
    }


    /*
     * Field-Centric Driving
     */

    public void driveVectorField(double axial, double lateral, double yaw, double power, IMUW imu) {
        // all math is done in radians, not degrees
        double angle = Math.atan2(axial, lateral);
        double r = Math.hypot(axial, lateral); // same as sqrt(axial^2 + lateral^2)

        // "rotate" the angle we want to go in by the robot;
        // eg. we want to go 0 degrees, robot is 90 degrees to right
        // so we really want to go 0-90= -90 degrees, relative to robot
        // doing += because my getYaw method negates output so cw is + and ccw is -
        angle += imu.getYaw(AngleUnit.RADIANS);
        // wrap radians, like -180 to 180 degrees but for radians
        angle = AngleUnit.normalizeRadians(angle);

        // take angle and do trig to convert back to vector parts (aka back to cartesian)
        double newAxial = r * Math.sin(angle);
        double newLateral = r * Math.cos(angle);

        logger.logData(Logger.LoggerMode.DETAILED, "finalCAngle", angle);
        logger.logData(Logger.LoggerMode.DETAILED, "newAxial", newAxial);
        logger.logData(Logger.LoggerMode.DETAILED, "newLateral", newLateral);
        logger.update();

        // yaw remains unchanged in robot vs field centric
        driveVector(newAxial, newLateral, yaw, power);
    }
}
