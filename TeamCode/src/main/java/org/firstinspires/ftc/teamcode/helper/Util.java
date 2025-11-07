package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Util {
    public static double angleWrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }
    /**
     * Clips a value to be within a minimum and maximum range.
     */
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static void sleepThread(int millis){
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
        }
    }

    public static void printAllOdoTelemetry(GoBildaPinpointDriver odo, Telemetry telemetry ){
        telemetry.addData("Odo Device Name: ", odo.getDeviceName());
        telemetry.addData("Odo Device Version: ", odo.getDeviceVersion());
        telemetry.addData("Odo Connection Info: ", odo.getConnectionInfo());
        telemetry.addData("Odo Device Status: ", odo.getDeviceStatus());
        telemetry.addData("Odo Position X (cms): ", odo.getPosX(DistanceUnit.CM));
        telemetry.addData("Odo Position Y (cms): ", odo.getPosY(DistanceUnit.CM));
        telemetry.addData("Odo Heading (degrees): ", odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    public static void printOdoPositionTelemetry( GoBildaPinpointDriver odo, Telemetry telemetry ){
        telemetry.addData("Odo Position X (cms): ", odo.getPosX(DistanceUnit.CM));
        telemetry.addData("Odo Position Y (cms): ", odo.getPosY(DistanceUnit.CM));
        telemetry.addData("Odo Heading (degrees): ", odo.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    public static void printIMUTelemetry(IMU imu, Telemetry telemetry ){
        telemetry.addData("IMU Device Name: ", imu.getDeviceName());
        telemetry.addData("IMU Device Version: ", imu.getVersion());
        telemetry.addData("IMU Connection Info: ", imu.getConnectionInfo());
        telemetry.addData("IMU Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("IMU Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("IMU Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
    }



    /**
     * Turns the robot to a specific angle using the IMU Yaw.
     * This is a self-contained method with no external utility calls.
     *
     * @param targetAngle The absolute angle in degrees to turn to (-180 to 180).
     * @param turnPower   The power at which to turn (0.0 to 1.0).
     * @param leftFront   The front left drive motor.
     * @param leftBack    The back left drive motor.
     * @param rightFront  The front right drive motor.
     * @param rightBack   The back right drive motor.
     * @param imu         The IMU sensor.
     * @param opMode      The LinearOpMode instance, used to check if the OpMode is still active.
     * @param telemetry   The telemetry object for displaying debug information.
     */
    public static void turnToAngleIMU(double targetAngle, double turnPower,
                             DcMotor leftFront, DcMotor leftBack,
                             DcMotor rightFront, DcMotor rightBack,
                             IMU imu, LinearOpMode opMode, Telemetry telemetry) {

        // Ensure the opMode is still active before starting.
        if (!opMode.opModeIsActive()) {
            return;
        }

        // --- Constants ---
        // How close to the target angle we need to be to stop.
        final double HEADING_TOLERANCE = 1.0; // degrees

        // --- Main Turn Logic ---

        // Reset the IMU's Yaw angle to 0. This makes the robot's current heading the new reference point.
        // All turns will be relative to this new zero heading.
        imu.resetYaw();

        // Get the current robot orientation from the IMU.
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

        // Calculate the error, which is the difference between where we want to be and where we are.
        double error = targetAngle - currentAngle;

        // Loop until the error is within our tolerance and the OpMode is still running.
        while (opMode.opModeIsActive() && Math.abs(error) > HEADING_TOLERANCE) {
            // This logic handles "wrap-around" on the -180 to 180 degree range.
            // For example, turning from -170 to 170 degrees should be a 20-degree right turn, not a 340-degree left turn.
            if (error > 180) {
                error -= 360;
            }
            if (error < -180) {
                error += 360;
            }

            // Determine the turn direction based on the sign of the error.
            // If error is positive, we need to turn left (counter-clockwise).
            // If error is negative, we need to turn right (clockwise).
            double directionSign = Math.signum(error);

            // Set the power to the motors. Left motors get negative power for a left turn,
            // and positive for a right turn. Right motors do the opposite.
            leftFront.setPower(-turnPower * directionSign);
            leftBack.setPower(-turnPower * directionSign);
            rightFront.setPower(turnPower * directionSign);
            rightBack.setPower(turnPower * directionSign);

            // --- Telemetry and Loop Update ---
            // Display debug information on the Driver Station.
            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Current Angle", "%.2f", currentAngle);
            telemetry.addData("Error", "%.2f", error);
            telemetry.update();

            // Re-calculate the error for the next loop iteration.
            orientation = imu.getRobotYawPitchRollAngles();
            currentAngle = orientation.getYaw(AngleUnit.DEGREES);
            error = targetAngle - currentAngle;
        }

        // --- Stop the Robot ---
        // Once the loop is finished, stop all drive motors.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public static void addKickerTelemetry(Kicker kicker, Telemetry telemetry){
        telemetry.addData("kicker.getPosition() - ",kicker.getPosition());
        telemetry.addData("kicker.getGatePosition() - ",kicker.getGatePosition());
    }

    public static void telemetryFlyWheelVelocity(FlyWheel flyWheel, double flyWheelPower, DistanceSensor frontDistanceSensor,  int runForMS, Telemetry telemetry){

        telemetry.addData("Power - "+ flyWheelPower, "Distance - " + Util.getDistance(frontDistanceSensor, telemetry));
        long startTime = System.currentTimeMillis();
        long intermidiateTime =  System.currentTimeMillis();
        long endTime = 0;
        long durationInMillis = intermidiateTime - startTime;

        double flyWheelVelocity = 0.0;

        flyWheel.setPower(flyWheelPower);
        //telemetry.addData("Flywheel StartTime: ", startTime);
        while(durationInMillis <= runForMS){
            intermidiateTime =  System.currentTimeMillis();
            durationInMillis = intermidiateTime - startTime;
            flyWheelVelocity = flyWheel.getVelocity();
            //telemetry.addData("Flywheel Intermidiate Time (ms): ", durationInMillis);
            telemetry.addData("Flywheel Velocity: " + flyWheelVelocity +" in time(ms): ", durationInMillis);
            sleepThread(1000);
        }
        endTime = System.currentTimeMillis();
        durationInMillis = endTime - startTime;

        telemetry.addData("Flywheel Total Time: ", durationInMillis);

        flyWheel.stop();
    }

    public static Long waitForFlyWheelShootingVelocity(FlyWheel flyWheel, long velocity, double maxWaitTime, Telemetry telemetry){
        long startTime = System.currentTimeMillis();
        long intermidiateTime =  System.currentTimeMillis();
        long durationInMillis = intermidiateTime - startTime;
        int loopCounter = 0;
        while (flyWheel.getVelocity() < 0.9*velocity){          //90% is good
            loopCounter++;
            intermidiateTime =  System.currentTimeMillis();
            durationInMillis = intermidiateTime - startTime;
            sleepThread(250);
            if(durationInMillis > maxWaitTime){
                return durationInMillis;
            }
        }
        telemetry.addData("waitForFlyWheelShootingVelocity - loopCounter - ",loopCounter);
        return durationInMillis;
    }

    public static long waitForFlyWheelStopVelocity(FlyWheel flyWheel, long velocity, double maxWaitTime, Telemetry telemetry){

        long startTime = System.currentTimeMillis();
        double startFlyWheelVelocity = flyWheel.getVelocity();
        double currentFlyWheelVelocity = flyWheel.getVelocity();

        telemetry.addData("Start FlyWheel Velocity - " , startFlyWheelVelocity);

        while (currentFlyWheelVelocity > velocity){
            flyWheel.setPower(-0.1);
            sleepThread(500);
            flyWheel.setPower(0.0);
            currentFlyWheelVelocity = flyWheel.getVelocity();
        }

        long endTime = System.currentTimeMillis();

        long durationInMillis = (endTime - startTime);
        double endFlyWheelVelocity = flyWheel.getVelocity();
        telemetry.addData("flyWheelVelocity - " + endFlyWheelVelocity,", durationInMillis - "+durationInMillis);
        return durationInMillis;

        /*
        long startTime = System.currentTimeMillis();
        long intermidiateTime =  System.currentTimeMillis();
        long durationInMillis = intermidiateTime - startTime;
        double flyWheelVelocity = flyWheel.getVelocity();
        telemetry.addData("Start flyWheelVelocity - ",flyWheelVelocity);

        while (flyWheelVelocity > velocity){
            intermidiateTime =  System.currentTimeMillis();
            durationInMillis = intermidiateTime - startTime;
            flyWheel.setPower(0.1);
            sleepThread(500);
            flyWheel.setPower(0.0);
            flyWheelVelocity = flyWheel.getVelocity();
            telemetry.addData("flyWheelVelocity - " + flyWheelVelocity,", durationInMillis - "+durationInMillis);
            //if(durationInMillis > maxWaitTime){
            //    return durationInMillis;
            //}
            durationInMillis = intermidiateTime - startTime;
        }
        return durationInMillis;

         */
    }

    public static boolean waitForMotor(DcMotorEx dcWheel, long timeoutMs, double ticksPerSecond) {
        boolean ret = false;
        long sleepInMs = 200;

        for (long i = 0; i < timeoutMs; i++) {
            if (dcWheel.getVelocity() >= Math.abs(ticksPerSecond)) {
                ret = true;
                break;
            }

            try {
                Thread.sleep(sleepInMs);
            } catch (InterruptedException e) {
            }

            i = i + sleepInMs;
        }

        return ret;
    }
    public static boolean isObjectDetected(DistanceSensor channelSensor, Telemetry telemetry) {
        boolean ret = false;

        // Get distance reading from 2M sensor
        double dDistance = channelSensor.getDistance(DistanceUnit.INCH);
        //telemetry.addData("distanceSensor - ",dDistance);
        //telemetry.update();
        // Check if distance is less than 10 cm
        if (dDistance < 14/2.54 ) ret = true;

        return ret;
    }

    public static double getDistance(DistanceSensor distanceSensor, Telemetry telemetry) {

        double dDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        return dDistance;
    }

    public static void prepareFlyWheelToShoot(FlyWheel flyWheel, Kicker kicker, Intake intake, Double distance, Telemetry telemetry){
        kicker.setPosition(Kicker.gateClose);

        intake.setIntakePower(0.5); //reduce intake power to avoid jam

        threadSleep(800);
        //Replace the timer with distance sensor
        //When the ball is moved out, start the flywheel
        flyWheel.start(Util.getRequiredFlyWheelPower(distance));
        //threadSleep(800);
    }

    public static void prepareFlyWheelToIntake(FlyWheel flyWheel, Kicker kicker, Intake intake, Flipper flipper, Telemetry telemetry){
        flyWheel.stop();
        kicker.setGatePosition(Kicker.GATE_CLOSE);
        flipper.resetFlipper();
        Util.waitForFlyWheelStopVelocity(flyWheel,200,5000, telemetry);
        kicker.setPosition(Kicker.gateIntake);
        intake.startIntake();
    }


    public static void startShooting(FlyWheel flyWheel, Kicker kicker, Flipper flipper, Intake intake, DistanceSensor channelDistanceSensor, Double distanceInInchFromAprilTag, Telemetry telemetry){
        Double requiredFlyWheelPower = Util.getRequiredFlyWheelPower(distanceInInchFromAprilTag);
        Integer requiredFlyWheelVelocity = Util.getRequiredFlyWheelVelocity(distanceInInchFromAprilTag);

        Long timeToReachRequiredFlyWheelVelocity = Util.waitForFlyWheelShootingVelocity(flyWheel,requiredFlyWheelVelocity,3000, telemetry);

        telemetry.addData("timeToReachRequiredFlyWheelVelocity - ",timeToReachRequiredFlyWheelVelocity);
      //  kicker.setPosition(Kicker.gateShoot);
       // sleepThread(200);

       /*
        if(!isObjectDetected(channelDistanceSensor, telemetry)){
            flipper.turnFlipper();
            sleepThread(200);
            kicker.setPosition(Kicker.gateClose);

        }



        if ( isObjectDetected(channelDistanceSensor, telemetry) ) {
            kicker.setPosition(Kicker.gateClose);
        }
        */


        //boolean isObjectDetected = isObjectDetected(channelDistanceSensor, telemetry);

        /*
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while ( !isObjectDetected(channelDistanceSensor, telemetry) ) {
            sleepThread(100);
            if (timer.milliseconds() > 1000) {
                break;
            }
        }

         */
        //if (isObjectDetected) {



    }

    public static Double getRequiredFlyWheelPower(double distanceInInchFromAprilTag){
        Double doubleDesiredFlyWheelPower = 0.45;

        if (distanceInInchFromAprilTag >= 25 && distanceInInchFromAprilTag <= 50){
            doubleDesiredFlyWheelPower = 0.45;
        }
        else if (distanceInInchFromAprilTag>50 && distanceInInchFromAprilTag<=70){
            doubleDesiredFlyWheelPower = 0.55;
        }
        else if (distanceInInchFromAprilTag>70 && distanceInInchFromAprilTag<=85){
            doubleDesiredFlyWheelPower = 0.65;
        }

        return doubleDesiredFlyWheelPower;
    }

    public static Integer getRequiredFlyWheelVelocity(Double distanceInInchFromAprilTag){
        Integer integerDesiredFlyWheelVelocity = 1200;

        if (distanceInInchFromAprilTag >= 25 && distanceInInchFromAprilTag <= 50){
            integerDesiredFlyWheelVelocity = 1200;
        }
        else if (distanceInInchFromAprilTag>50 && distanceInInchFromAprilTag<=70){
            integerDesiredFlyWheelVelocity = 1300;
        }
        else if (distanceInInchFromAprilTag>70 && distanceInInchFromAprilTag<=85){
            integerDesiredFlyWheelVelocity = 1400;
        }

        return integerDesiredFlyWheelVelocity;
    }


    /*
    public static int getRequiredFlyWheelVelocityRequiredForDistance(Double distanceInInchFromAprilTag){
        int desiredFlyWheelVelocity = FlyWheel.FLYWHEEL_SHOOTING_VELOCITY;

        Double doubleDesiredFlyWheelPower = 0.004*(distanceInInchFromAprilTag)+0.399;
        Double doubleDesiredFlyWheelVelocity = (doubleDesiredFlyWheelPower * 1500)/0.65;
        doubleDesiredFlyWheelPower.compareTo()


         desiredFlyWheelVelocity = doubleDesiredFlyWheelVelocity.intValue();


        return desiredFlyWheelVelocity;
    }

     */

    public static double getDistanceRequiredForFlyWheelVelocity(Integer flyWheelVelocity){
        double distanceInInchFromAprilTag = 53.0;

        return distanceInInchFromAprilTag;
    }



    public static void threadSleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            return;
        }
    }

    }
