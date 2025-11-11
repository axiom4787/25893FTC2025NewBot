package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
//        telemetry.addData("Odo Heading (degrees): ", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odo Heading (radians): ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.update();
    }

    public static void printOdoPositionTelemetry( GoBildaPinpointDriver odo, Telemetry telemetry ){
        odo.update();

        Pose2D pose = odo.getPosition();

        telemetry.addData("Odo Encoder X : ", odo.getEncoderX());
        telemetry.addData("Odo Encoder Y : ", odo.getEncoderY());
        telemetry.addData("Odo Pose X (cms): ", pose.getX(DistanceUnit.CM));
        telemetry.addData("Odo Pose Y (cms): ", pose.getY(DistanceUnit.CM));
        telemetry.addData("Odo Position X (cms): ", odo.getPosX(DistanceUnit.CM));
        telemetry.addData("Odo Position Y (cms): ", odo.getPosY(DistanceUnit.CM));
        //telemetry.addData("Odo Heading (degrees): ", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odo Heading (radians): ", odo.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Odo Heading (rad to deg): ", Math.toDegrees(odo.getHeading(AngleUnit.RADIANS)));

    }

    public static void printIMUTelemetry(IMU imu, Telemetry telemetry ){
        //telemetry.addData("IMU Device Name: ", imu.getDeviceName());
        //telemetry.addData("IMU Device Version: ", imu.getVersion());
        //telemetry.addData("IMU Connection Info: ", imu.getConnectionInfo());

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

        double shootPower = flyWheel.getPower();
        flyWheel.setPower(1.0); // ramp up at full power

        while (flyWheel.getVelocity() < 0.98*velocity){
            loopCounter++;
            intermidiateTime =  System.currentTimeMillis();
            durationInMillis = intermidiateTime - startTime;
            sleepThread(50);

            if(durationInMillis > maxWaitTime){
                return durationInMillis;
            }
        }

        while (flyWheel.getVelocity() > 1.02*velocity){
            flyWheel.setPower(-1);
            sleepThread(50);
        }

        flyWheel.setPower((0.45 + (velocity-1200)/1000)*1.2); //20% more for shooting power

     //   flyWheel.setPower(shootPower); // back to shooting power

        telemetry.addData("waitForFlyWheelShootingVelocity - loopCounter - ",loopCounter);
        return durationInMillis;
    }

    public static long waitForFlyWheelStopVelocity(FlyWheel flyWheel, long velocity, double maxWaitTime, Telemetry telemetry){

        long startTime = System.currentTimeMillis();
        double startFlyWheelVelocity = flyWheel.getVelocity();
        double currentFlyWheelVelocity = flyWheel.getVelocity();

        telemetry.addData("Start FlyWheel Velocity - " , startFlyWheelVelocity);

        //flyWheel.setPower(-1); // was -0.1
        //sleepThread(500);
        //flyWheel.setPower(0.0);

        while (currentFlyWheelVelocity > velocity){
            flyWheel.setPower(-1); // was -0.1
            sleepThread(50);
           // flyWheel.setPower(0.0);
            currentFlyWheelVelocity = flyWheel.getVelocity();
        }

        flyWheel.setPower(0.0);

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

        intake.setIntakePower(0.5); //reduce intake power to avoid jam
        kicker.setPosition(Kicker.gateClose);
        threadSleep(200);

        //threadSleep(800);
        //Replace the timer with distance sensor
        //When the ball is moved out, start the flywheel


       // flyWheel.start(0.5); // start at low power


        // flyWheel.start(Util.getRequiredFlyWheelPower(distance));
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


    public static void prepareForShooting(FlyWheel flyWheel, Kicker kicker, Flipper flipper, Intake intake, Double distanceInInchFromAprilTag, Telemetry telemetry){
        //Double requiredFlyWheelPower = Util.getRequiredFlyWheelPower(distanceInInchFromAprilTag);
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
        Integer integerDesiredFlyWheelVelocity;

        /* measurement data (velocity_rpm vs distance_inch)
        [950 1000 1100 1200 1300 1400] rpm
        [37  43.5    51 54.5 67.5 77] - inch

        velocity = 10.25*distance + 587.4, R^2=0.992

        minmum shooting distance is 37 inch
         */

        integerDesiredFlyWheelVelocity = (int) Math.max(950, Math.ceil(10.25*distanceInInchFromAprilTag + 587.4));


        /*
        if (distanceInInchFromAprilTag >= 25 && distanceInInchFromAprilTag <= 50){
            integerDesiredFlyWheelVelocity = 1200;
        }
        else if (distanceInInchFromAprilTag>50 && distanceInInchFromAprilTag<=70){
            integerDesiredFlyWheelVelocity = 1300;
        }
        else if (distanceInInchFromAprilTag>70 && distanceInInchFromAprilTag<=85){
            integerDesiredFlyWheelVelocity = 1400;
        }

         */
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

    public static void shoot(FlyWheel flyWheel, Kicker kicker, Flipper flipper, Intake intake, Double robotDistanceFromAprilTag, Telemetry telemetry){

        prepareFlyWheelToShoot(flyWheel, kicker, intake, robotDistanceFromAprilTag, telemetry);

        // intake.setIntakePower(0.5);

        int loopCounter = 0;
        double flipperangle = 120;

        while(loopCounter<4) {

            // wait flywheel to get the desired speed
            Util.prepareForShooting(flyWheel, kicker, flipper, intake, robotDistanceFromAprilTag, telemetry);

            kicker.setPosition(Kicker.gateShoot);
            threadSleep(300);

            flipper.turnFlipper(flipperangle+loopCounter*30);
            threadSleep(150 + loopCounter*50);
            kicker.setGatePosition(Kicker.GATE_CLOSE);
            flipper.resetFlipper();
            threadSleep(200);


            loopCounter = loopCounter+1;

            //  sleep(200);

        }
        telemetry.addData( "loopCounter - ",loopCounter);
        telemetry.update();

        //flipper.resetFlipper();


        Util.prepareFlyWheelToIntake(flyWheel,kicker,intake,flipper, telemetry);


    }

    public enum MovementDirection {
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TURN_LEFT,
        TURN_RIGHT
    }



    public static void moveRobot(
            DcMotor leftFront, DcMotor leftBack,
            DcMotor rightFront, DcMotor rightBack,
            GoBildaPinpointDriver odo,
            IMU imu,
            MovementDirection direction,
            double distance,
            double targetAngle,
            Telemetry telemetry) {


        Double MAX_SPEED = 0.8;
        Double MIN_SPEED = 0.2;
        Double P_DRIVE_GAIN = 0.03;
        Double ERROR_RANGE__INCHES = 0.5;
        Double P_YAW_GAIN = 0.10;
        Double ERROR_RANGE_DEGREE = 2.0;
        Integer TIMEOUT_MS = 5000;

        // --- 1. Initialization ---
        odo.resetPosAndIMU();
        odo.setPosX(0.0,DistanceUnit.INCH);
        odo.setPosY(0.0,DistanceUnit.INCH);
        threadSleep(100);

        odo.update();
        Double startX = odo.getPosX(DistanceUnit.INCH);
        Double startY = odo.getPosY(DistanceUnit.INCH);
        Double startYaw = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
        //Double currentAngleRAD = odo.getHeading(AngleUnit.RADIANS);

        ElapsedTime timer = new ElapsedTime();

        // --- We will use different variables depending on direction ---
        Double targetPosition;
        Double currentPosition;
        Double currentHeading;
        Double currentAngle;

        Double errorDistance;
        Double errorAngle;

        Double axialPower;
        Double lateralPower;
        Double yawPower;





        // --- 2. ISOLATED Control Loop ---
        // We run a DIFFERENT loop based on the direction.
        // This prevents the robot from trying to do two things at once.

        switch (direction) {
            case FORWARD:
                targetPosition = startY + distance;

                // --- TELEMETRY 1: BEFORE THE LOOP ---
                // (Checks starting values and calculation)
                // odo.updat();
//                Double odoPosX = odo.getPosX(DistanceUnit.INCH);
//                Double odoPosY = odo.getPosY(DistanceUnit.INCH);

                /*
                telemetry.addData("--- DEBUG: PRE-RUN ---", "");
                telemetry.addData("Direction", direction);
                telemetry.addData("Start X", "%.2f", startX);
                telemetry.addData("Target X", "%.2f", targetPosition);
                telemetry.addData("Distance", "%.2f", distance);
                telemetry.addData("currentPosition", "%.2f", currentPosition);
                telemetry.addData("odoPosX", "%.2f", odoPosX);
                telemetry.addData("odoPosY", "%.2f", odoPosY);
                telemetry.addData("Tgt Heading", "%.2f", targetHeadingRadians);
                telemetry.addData("Start Heading", "%.2f", currentHeading);
                telemetry.update();
                threadSleep(3000);
                */

                timer = new ElapsedTime();
                int loopCounter = 0;
                // Keep looping until we pass the target
                odo.update();
                while (odo.getPosY(DistanceUnit.INCH) < targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    loopCounter++;
                    odo.update();

                    // --- A. Drive (Axial) Calculation ---
                    currentPosition = odo.getPosY(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition;

                    // P-Controller for Axial
                    axialPower = Range.clip(errorDistance * P_DRIVE_GAIN, MIN_SPEED, MAX_SPEED);


                    // --- B. Heading (Yaw) Calculation ---
                    currentHeading = odo.getHeading(AngleUnit.RADIANS);
                    double headingError = targetAngle - currentHeading;

                    // Handle wraparound (e.g., target 1°, current 359°)
                    while (headingError > Math.PI)  headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;

                    // P-Controller for Yaw
                    yawPower = Range.clip(headingError * P_YAW_GAIN, -MAX_SPEED, MAX_SPEED);

                    // Apply *only* axial (forward) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, axialPower, 0, yawPower);


                    // --- TELEMETRY 2: INSIDE THE LOOP ---
                    // (This is existing, correct telemetry)
                    odo.update();

                }
                //Break after the movement.
                setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);
                //threadSleep(5000);
                break;

            case BACKWARD:
                targetPosition = startY - distance;

                // Keep looping until we pass the target
                while (odo.getPosY(DistanceUnit.INCH) > targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    odo.update();
                    currentPosition = odo.getPosX(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition; // Error will be negative

                    // Simple P-Controller. Note: errorDistance * gain will be negative.
                    axialPower = Range.clip(errorDistance * P_DRIVE_GAIN, -MAX_SPEED, -MIN_SPEED);

                    // Apply *only* axial (backward) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, axialPower, 0, 0);

                    telemetry.addData("Moving", direction);
                    telemetry.addData("Target", "%.2f", targetPosition);
                    telemetry.addData("Current", "%.2f", currentPosition);
                    telemetry.addData("Power", "%.2f", axialPower);
                    telemetry.update();
                    odo.update();
                }
                break;

            case STRAFE_RIGHT:
                targetPosition = startX + distance;

                // Keep looping until we pass the target
                while (odo.getPosX(DistanceUnit.INCH)< targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    odo.update();
                    currentPosition = odo.getPosX(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition;

                    // Simple P-Controller
                    lateralPower = Range.clip(errorDistance * P_DRIVE_GAIN, MIN_SPEED, MAX_SPEED);

                    // Apply *only* lateral (strafe) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, -lateralPower, 0);

                    telemetry.addData("Moving", direction);
                    telemetry.addData("Target", "%.2f", targetPosition);
                    telemetry.addData("Current", "%.2f", currentPosition);
                    telemetry.addData("Power", "%.2f", lateralPower);
                    telemetry.update();
                    odo.update();
                }
                break;

            case STRAFE_LEFT:
                targetPosition = startX - distance;

                // Keep looping until we pass the target
                while (odo.getPosX(DistanceUnit.INCH) > targetPosition && timer.milliseconds() < TIMEOUT_MS) {
                    odo.update();
                    currentPosition = odo.getPosX(DistanceUnit.INCH);
                    errorDistance = targetPosition - currentPosition; // Error will be negative

                    // Simple P-Controller
                    lateralPower = Range.clip(errorDistance * P_DRIVE_GAIN, -MAX_SPEED, -MIN_SPEED);

                    // Apply *only* lateral (strafe) power
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, -lateralPower, 0);

                    telemetry.addData("Moving", direction);
                    telemetry.addData("Target", "%.2f", targetPosition);
                    telemetry.addData("Current", "%.2f", currentPosition);
                    telemetry.addData("Power", "%.2f", lateralPower);
                    telemetry.update();

                    odo.update();
                }
                break;
            case TURN_LEFT:
                odo.update();
                currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                telemetry.addData("Target Angle","%.2f", targetAngle);
                telemetry.addData("Current Angle","%.2f", currentAngle);
                telemetry.addData("Error Angle", "%.2f",errorAngle);
                telemetry.update();
                sleepThread(1000);

                //while(Math.abs(errorAngle) < ERROR_RANGE_DEGREE && timer.milliseconds() < TIMEOUT_MS) {

                while(currentAngle < targetAngle && timer.milliseconds() < TIMEOUT_MS) {

                    Double turnPower = ((Math.abs(errorAngle) / 180) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED) * Math.signum(errorAngle) * -1;

                    telemetry.addData("Target Angle","%.2f", targetAngle);
                    telemetry.addData("Current Angle","%.2f", currentAngle);
                    telemetry.addData("Error Angle", "%.2f",errorAngle);
                    telemetry.addData("Turn Power", "%.2f",turnPower);
                    telemetry.update();

                    //Half turn speed
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, -turnPower);

                    odo.update();
                    //currentAngleRAD = odo.getHeading(AngleUnit.RADIANS);
                    currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                    errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                    if( Math.abs(errorAngle) < ERROR_RANGE_DEGREE ) {
                        Util.setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);
                        break;
                    }
                }
                break;
            case TURN_RIGHT:
                odo.update();
                currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                telemetry.addData("Target Angle","%.2f", targetAngle);
                telemetry.addData("Current Angle","%.2f", currentAngle);
                telemetry.addData("Error Angle", "%.2f",errorAngle);
                telemetry.update();
                sleepThread(1000);

                //while(Math.abs(errorAngle) < ERROR_RANGE_DEGREE && timer.milliseconds() < TIMEOUT_MS) {

                while(currentAngle > targetAngle && timer.milliseconds() < TIMEOUT_MS) {

                    Double turnPower = ((Math.abs(errorAngle) / 180) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED) * Math.signum(errorAngle) * -1;

                    telemetry.addData("Target Angle","%.2f", targetAngle);
                    telemetry.addData("Current Angle","%.2f", currentAngle);
                    telemetry.addData("Error Angle", "%.2f",errorAngle);
                    telemetry.addData("Turn Power", "%.2f",turnPower);
                    telemetry.update();

                    //Half turn speed
                    setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, -turnPower);

                    odo.update();
                    //currentAngleRAD = odo.getHeading(AngleUnit.RADIANS);
                    currentAngle = Math.toDegrees(AngleUnit.normalizeRadians(odo.getHeading(AngleUnit.RADIANS)));
                    errorAngle = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

                    if( Math.abs(errorAngle) < ERROR_RANGE_DEGREE ) {
                        Util.setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);
                        break;
                    }
                }
                break;

        }

        // --- 3. Stop Motors ---
        setMotorPower(leftFront, leftBack, rightFront, rightBack, 0, 0, 0);

//        telemetry.addData("Status", "Movement Complete");
//        telemetry.addData("Final X", "%.2f", odo.getPosX(DistanceUnit.INCH));
//        telemetry.addData("Final Y", "%.2f", odo.getPosY(DistanceUnit.INCH));
//        telemetry.update();
    }

    /**
     * Helper method to apply power to all four motors using Mecanum logic.
     * (This is your original 'drive' logic, simplified)
     *
     * @param axialPower   Forward/Backward power (+/-)
     * @param lateralPower Strafe Left/Right power (+/-)
     * @param yawPower     Rotation power (+/-)
     */
    public static void setMotorPower(
            DcMotor leftFront, DcMotor leftBack,
            DcMotor rightFront, DcMotor rightBack,
            double axialPower, double lateralPower, double yawPower) {

        double leftFrontPower = axialPower - lateralPower - yawPower;
        double rightFrontPower = axialPower + lateralPower + yawPower;
        double leftBackPower = axialPower + lateralPower - yawPower;
        double rightBackPower = axialPower - lateralPower + yawPower;

        // Normalize (same as before)
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    }
