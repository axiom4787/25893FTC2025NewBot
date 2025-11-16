package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Autonomous(name = "week1Auto", group = "Competition")
public class week1Auto extends LinearOpMode {

    // Drive motors
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    
    // Other motors
    DcMotorEx intake, shooter, transfer;
    
    // Servos
    Servo hold;

    Limelight3A limelight;
    
    // Odometry
    private GoBildaPinpointDriver odo = null;
    
    // Voltage sensor for battery compensation
    private VoltageSensor batteryVoltageSensor = null;
    private double restingBatteryVoltage = 13.0; // Measured at rest before autonomous starts
    
    // Robot specifications
    private static final double MOTOR_PPR = 7.0;           // Pulses per revolution
    private static final double GEAR_RATIO = 12.7;         // Gear ratio
    private static final double WHEEL_DIAMETER_MM = 86.0;  // Wheel diameter in mm
    private static final double COUNTS_PER_MOTOR_REV = MOTOR_PPR * GEAR_RATIO;  // ~88.9
    private static final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;  // ~270.18 mm
    private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE_MM;  // ~0.329 counts/mm
    
    // Shooter parameters
    private static final int SHOOTER_TARGET_VELOCITY = 1400;
 
    private static final double NOMINAL_VOLTAGE = 13.6;
    private static final double MIN_VOLTAGE_CORRECTION = 0.80;  // Don't reduce power more than 15%
    private static final double MAX_VOLTAGE_CORRECTION = 1.20;  // Don't increase power more than 15%
    
    // Power ramping to prevent jerky starts
    private static final double MAX_POWER_CHANGE = 0.05;  // Max power change per loop iteration
    private double lastFrontLeftPower = 0;
    private double lastFrontRightPower = 0;
    private double lastBackLeftPower = 0;
    private double lastBackRightPower = 0;
    
    // Alliance selection
    private boolean isRedAlliance = true;  // Will be set based on gamepad input
    private int allianceMultiplier = 1;     // 1 for red, -1 for blue
    
    // Shooter state for background control
    private boolean shooterEnabled = false;

    @Override
    public void runOpMode() {
        
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- HARDWARE MAPPING ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        
        hold = hardwareMap.get(Servo.class, "blocker");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        // Initialize GoBilda Pinpoint odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.resetPosAndIMU();
        odo.setOffsets(60, 35, DistanceUnit.MM);
        odo.setEncoderResolution(10.45, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Initialize voltage sensor for battery compensation
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // --- MOTOR DIRECTION ---
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // --- MOTOR BEHAVIORS ---
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure shooter PIDF
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(53, 0.02, 3, 11);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize motors without encoder for odometry-based control
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Other motors without encoders
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servo
        hold.setPosition(1);
        
        // Reset encoders for odometry
        resetEncoders();
        
        // Reset Pinpoint heading
        odo.resetPosAndIMU();
        //odo.resetIMU();

        telemetry.addData("Status", "Initialized. Ready to run!");
        telemetry.update();
        
        // Alliance selection
        selectAlliance();

        waitForStart();

        if (opModeIsActive()) {
            
            // CRITICAL: Measure battery voltage AT REST before any movement
            // This avoids measuring voltage sag under load
            restingBatteryVoltage = batteryVoltageSensor.getVoltage();
            
            telemetry.addData("===== BATTERY INFO =====", "");
            telemetry.addData("Resting Voltage", "%.2fV", restingBatteryVoltage);
            telemetry.addData("Nominal Voltage", "%.2fV", NOMINAL_VOLTAGE);
            telemetry.addData("Voltage Correction", "%.3f", NOMINAL_VOLTAGE / restingBatteryVoltage);
            telemetry.addData("", "");
            telemetry.update();
            sleep(1000); // Show voltage for 1 second

            
            //shoot preloads
            setShooter(true);
            driveV2(.8, 11.5);  // ~3100mm converted to inches (3100 * 0.0393701)
            turnToTarget();
            shoot();
            setShooter(false);
            
            //drive to stack
            turnToImu(-110 * allianceMultiplier);
            driveV2(.8, 7);  // ~2250mm
            turnToImu(-40 * allianceMultiplier);
            setIntake(true);
            driveV2(.8, 8);  // ~2250mm
            turnToImu(-75 * allianceMultiplier);
            setIntake(false);
            
            
            setShooter(true);
            driveV2(.8, -14);  // ~2250mm
            turnToImu(180 * allianceMultiplier);
            turnToTarget();
            shoot();
            setShooter(false);
            //turnToImu(160 * allianceMultiplier);
            
            
            turnToImu(-113 * allianceMultiplier);
            // //intake stack and push bar
            driveV2(0.8, 15.3);  // ~3500mm
            turnToImu(-43 * allianceMultiplier);
            
            setIntake(true);
            driveV2(.3, 8.5);  // ~3500mm
            
            //driveV2(.5, -10);
            
            setShooter(true);
            turnToImu(-90 * allianceMultiplier);
            driveV2(.5, -15);
            setIntake(false);
            turnToImu(180 * allianceMultiplier);
            turnToTarget();
            shoot();
            setShooter(false);
            
            
            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    // ==================== ODOMETRY-BASED PATHING METHODS ====================
    
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getHeading() {
        odo.update();
        return odo.getHeading(AngleUnit.DEGREES) + 180;
    }
    
    // Test method to check if odometry is working
    public void testOdometry() {
        telemetry.addData("=== ODOMETRY TEST ===", "");
        telemetry.addData("Instructions", "Manually rotate robot and watch heading change");
        telemetry.addData("", "");
        
        for (int i = 0; i < 100 && opModeIsActive(); i++) {
            odo.update();
            telemetry.addData("Raw Heading", "%.2f", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Adjusted Heading (+180)", "%.2f", getHeading());
            telemetry.addData("Odo Status", odo.getDeviceStatus());
            telemetry.update();
            sleep(100);
        }
    }

    public double getLimelightTx(){
        LLResult result = limelight.getLatestResult();
        return result.getTx();
    }

    public void turnToTarget(){
        boolean isAligned = false;
        double targetTx = 0;
        double dkp = 0;
        double tx = 0;

        while (opModeIsActive() && !isAligned) {
            tx = getLimelightTx();
            double errorr = targetTx - tx;
            telemetry.addData("error of limelight: ", errorr);
            telemetry.addData("tx: ", tx);

            if(Math.abs(errorr) > 9) {
                dkp = 0.02;
            } else if(Math.abs(errorr) > 7) {
                dkp = 0.022;
            } else if(Math.abs(errorr) > 3) {
                dkp = 0.023;
            } else {
                dkp = 0.032;
            }
            
            double drivePower = errorr * dkp;
            
            frontLeft.setPower(-drivePower);
            frontRight.setPower(drivePower);
            backLeft.setPower(-drivePower);
            backRight.setPower(drivePower);
            
            telemetry.addData("drivePower: ", drivePower);
            telemetry.update();
            
            if(tx > -2 && tx < 2){
                isAligned = true;
            }
        }
        
        stopDrive();
    }

    public void turnToImu(double targetHeading) {
        double roboHeading, error;
    
        do {
            // Update odometry every loop iteration
            odo.update();
            roboHeading = getHeading();
            error = roboHeading - targetHeading;
            
            // Normalize error to be within -180 to 180 degrees
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
    
            // VOLTAGE COMPENSATION: Use resting voltage (measured before movement)
            // This avoids voltage sag under load causing incorrect corrections
            double voltageCorrection = 1.0; // Default
            if (restingBatteryVoltage > 0) { // Avoid division by zero
                voltageCorrection = NOMINAL_VOLTAGE / restingBatteryVoltage;
                // Clamp to prevent extreme corrections
                voltageCorrection = Math.max(MIN_VOLTAGE_CORRECTION, 
                                           Math.min(MAX_VOLTAGE_CORRECTION, voltageCorrection));
            }
    
            double p = 0.0075, kf = 0.0;
    
            if (error < 0) {
                kf = -0.18;
            } else if (error > 0) {
                kf = 0.18;
            }
    
            // Calculate raw power and apply voltage compensation
            double rightPower = -(p * error + kf);
            double leftPower = (p * error + kf);
            
            frontRight.setPower(rightPower * voltageCorrection);
            frontLeft.setPower(leftPower * voltageCorrection);
            backRight.setPower(rightPower * voltageCorrection);
            backLeft.setPower(leftPower * voltageCorrection);
            
            updateShooter();
    
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", roboHeading);
            telemetry.addData("Raw Heading (no +180)", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Error", error);
            telemetry.addData("Motor Power", frontLeft.getPower());
            telemetry.addData("Resting Voltage> ", "%.2fV", restingBatteryVoltage);
            telemetry.addData("Voltage Correction> ", "%.3f", voltageCorrection);
            telemetry.addData("Odo Status", odo.getDeviceStatus());
            telemetry.update();
        } while (opModeIsActive() && !(error < 0.5 && error > -0.5));
        
        stopDrive();
    }

    public void driveV2(double power, double targetY) {
        resetEncoders();
        //odo.recalibrateIMU();
    
        // BUG FIX 1: Correctly average all four motors (was using backRight twice)
        double average = (frontRight.getCurrentPosition() + backRight.getCurrentPosition() + 
                         backLeft.getCurrentPosition() + frontLeft.getCurrentPosition()) / 4;
        double roboY = (average) * (4 * 3.1415 / 1120);
        double roboHeading = getHeading();
    
        double targetHeading = 0;
        double kf;
    
        telemetry.addData("robo", roboY);
        telemetry.addData("Targ", targetY);
        telemetry.addData("average", average);
        telemetry.addData("flPower", frontLeft.getPower());
        telemetry.update();
    
        while (opModeIsActive() && (roboY > targetY + 0.2 || roboY < targetY - 0.2)) {
    
            // BUG FIX 1: Correctly average all four motors
            average = (frontRight.getCurrentPosition() + backRight.getCurrentPosition() + 
                      backLeft.getCurrentPosition() + frontLeft.getCurrentPosition()) / 4;
            roboY = (average) * (4 * 3.1415 / 1120);
    
            roboHeading = getHeading();
    
            double error = roboHeading - targetHeading;
            // Normalize error to -180 to 180 degrees
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            
            // VOLTAGE COMPENSATION: Use resting voltage (measured before movement)
            // This avoids voltage sag under load causing incorrect corrections
            double voltageCorrection = 1.0; // Default
            if (restingBatteryVoltage > 0) { // Avoid division by zero
                voltageCorrection = NOMINAL_VOLTAGE / restingBatteryVoltage;
                // Clamp to prevent extreme corrections
                voltageCorrection = Math.max(MIN_VOLTAGE_CORRECTION, 
                                           Math.min(MAX_VOLTAGE_CORRECTION, voltageCorrection));
            }
            
            // BUG FIX 2: Add a non-zero P gain for steering (was 0.000)
            double kp = 0.00; // START TUNING HERE - adjust as needed
    
            double a = error * kp;
            double errorD = targetY - roboY;
            double ep = 0.09;
    
            if (targetY > roboY) {
                kf = 0.12;
            } else {
                kf = -0.12;
            }
    
            // Calculate raw power
            double rightPower = (ep * errorD) * power - a + kf;
            double leftPower = (ep * errorD) * power + a + kf;
            
            // VOLTAGE COMPENSATION: Apply correction to final power
            frontRight.setPower(rightPower * voltageCorrection);
            backRight.setPower(rightPower * voltageCorrection);
            frontLeft.setPower(leftPower * voltageCorrection);
            backLeft.setPower(leftPower * voltageCorrection);
            
            updateShooter();
            
            telemetry.addData("CurrentHeading> ", roboHeading);
            telemetry.addData("TargetHeading> ", targetHeading);
            telemetry.addData("CurrentPos> ", roboY);
            telemetry.addData("TargetPos> ", targetY);
            telemetry.addData("average> ", average);
            telemetry.addData("fl power> ", frontLeft.getPower());
            telemetry.addData("Resting Voltage> ", "%.2fV", restingBatteryVoltage);
            telemetry.addData("Voltage Correction> ", "%.3f", voltageCorrection);
            telemetry.update();
        }
    
        stopDrive();
    }
    
    public void strafeV2(double power, double targetY) {
        resetEncoders();
    
        double average = (frontRight.getCurrentPosition() + -backRight.getCurrentPosition() + 
                         backLeft.getCurrentPosition() + -frontLeft.getCurrentPosition()) / 4;
        double roboY = (average) * (4 * 3.1415 / 1120);
    
        double roboHeading = getHeading();
        double targetHeading = roboHeading;
    
        double kf;
    
        telemetry.addData("robo", roboY);
        telemetry.addData("Targ", targetY);
        telemetry.addData("average", average);
        telemetry.addData("flPower", frontLeft.getPower());
        telemetry.update();
    
        while (opModeIsActive() && (roboY > targetY + 0.2 || roboY < targetY - 0.2)) {
    
            average = (frontRight.getCurrentPosition() + -backRight.getCurrentPosition() + 
                      backLeft.getCurrentPosition() + -frontLeft.getCurrentPosition()) / 4;
            roboY = (average) * (4 * 3.1415 / 1120);
    
            roboHeading = getHeading();
    
            double error = roboHeading - targetHeading;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            
            // VOLTAGE COMPENSATION: Use resting voltage (measured before movement)
            // This avoids voltage sag under load causing incorrect corrections
            double voltageCorrection = 1.0;
            if (restingBatteryVoltage > 0) { 
                voltageCorrection = NOMINAL_VOLTAGE / restingBatteryVoltage;
                // Clamp to prevent extreme corrections
                voltageCorrection = Math.max(MIN_VOLTAGE_CORRECTION, 
                                           Math.min(MAX_VOLTAGE_CORRECTION, voltageCorrection));
            }
            
            double kp = 0;//(0.0118 / 0.3);
    
            double a = error * kp;
            double errorD = targetY - roboY;
            double ep = 0.061;
    
            if (targetY > roboY) {
                kf = 0.3;
            } else {
                kf = -0.3;
            }
    
            double frPower = (ep * errorD) * power - a + kf;
            double brPower = (ep * errorD) * -power - a - kf;
            double flPower = (ep * errorD) * -power + a - kf;
            double blPower = (ep * errorD) * power + a + kf;
            
            frontRight.setPower(frPower * voltageCorrection);
            backRight.setPower(brPower * voltageCorrection);
            frontLeft.setPower(flPower * voltageCorrection);
            backLeft.setPower(blPower * voltageCorrection);
            
            updateShooter();
            
            telemetry.addData("Heading", roboHeading);
            telemetry.addData("robo", roboY);
            telemetry.addData("Targ", targetY);
            telemetry.addData("average", average);
            telemetry.addData("flPower", frontLeft.getPower());
            telemetry.addData("frPower", frontRight.getPower());
            telemetry.addData("blPower", backLeft.getPower());
            telemetry.addData("brPower", backRight.getPower());
            telemetry.addData("Resting Voltage> ", "%.2fV", restingBatteryVoltage);
            telemetry.addData("Voltage Correction> ", "%.3f", voltageCorrection);
            telemetry.update();
        }
    
        stopDrive();
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        
        // Reset ramp tracking
        lastFrontLeftPower = 0;
        lastFrontRightPower = 0;
        lastBackLeftPower = 0;
        lastBackRightPower = 0;
    }
    
    /**
     * Apply power with ramping to prevent jerky movements
     */
    private void setMotorPowersRamped(double flTarget, double frTarget, double blTarget, double brTarget) {
        // Ramp power gradually
        lastFrontLeftPower = rampPower(lastFrontLeftPower, flTarget);
        lastFrontRightPower = rampPower(lastFrontRightPower, frTarget);
        lastBackLeftPower = rampPower(lastBackLeftPower, blTarget);
        lastBackRightPower = rampPower(lastBackRightPower, brTarget);
        
        // Apply ramped power
        frontLeft.setPower(lastFrontLeftPower);
        frontRight.setPower(lastFrontRightPower);
        backLeft.setPower(lastBackLeftPower);
        backRight.setPower(lastBackRightPower);
    }
    
    /**
     * Ramp power value gradually
     */
    private double rampPower(double current, double target) {
        double delta = target - current;
        if (Math.abs(delta) > MAX_POWER_CHANGE) {
            return current + Math.signum(delta) * MAX_POWER_CHANGE;
        }
        return target;
    }

    // ==================== SUBSYSTEM CONTROL METHODS ====================

    /**
     * Turn intake on or off
     * @param on True to turn on, false to turn off
     */
    private void setIntake(boolean on) {
        if (on) {
            intake.setPower(-1.0);
            transfer.setPower(1.0);
            telemetry.addData("Intake", "ON");
        } else {
            intake.setPower(0);
            transfer.setPower(0);
            telemetry.addData("Intake", "OFF");
        }
        telemetry.update();
    }

    /**
     * Turn shooter on or off (non-blocking)
     * @param on True to enable shooter, false to disable
     */
    private void setShooter(boolean on) {
        shooterEnabled = on;
        if (!on) {
            shooter.setPower(0);
            telemetry.addData("Shooter", "OFF");
            telemetry.update();
        } else {
            // Spin up shooter for a moment when enabled
            telemetry.addData("Shooter", "Spinning up...");
            telemetry.update();
            for (int i = 0; i < 20; i++) {
                updateShooter();
                sleep(5);
            }
            telemetry.addData("Shooter", "Ready");
            telemetry.update();
        }
    }
    
    /**
     * Update shooter power based on velocity (bang-bang control)
     * Call this continuously during autonomous to maintain shooter speed
     */
    private void updateShooter() {
        if (shooterEnabled) {
            double currentVelocity = shooter.getVelocity();
            
            // Bang-bang control: full power if below target, reduced power if at/above target
            if (currentVelocity < SHOOTER_TARGET_VELOCITY) {
                shooter.setPower(1.0);
            } else {
                shooter.setPower(0.55);
            }
        }
    }

    /**
     * Execute a shot (waits for shooter to be ready if needed)
     */
    private void shoot() {
        // Wait for shooter to reach target velocity
        hold.setPosition(0.82);
        while (opModeIsActive() && shooterEnabled && shooter.getVelocity() < SHOOTER_TARGET_VELOCITY) {
            updateShooter();
            telemetry.addData("Action", "Waiting for shooter...");
            telemetry.addData("Shooter Velocity", "%.0f / %d", shooter.getVelocity(), SHOOTER_TARGET_VELOCITY);
            telemetry.update();
            sleep(50);
        }
        
        telemetry.addData("Action", "Shooting...");
        telemetry.update();
        
        // Release hold servo and run transfer
        hold.setPosition(0.82);
        transfer.setPower(0.5);
        intake.setPower(-1.0);
        
        // Wait for shot to complete, continue updating shooter
        long shotStartTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - shotStartTime) < 2000) {
            updateShooter();
            sleep(50);
        }
        
        // Reset to hold position
        hold.setPosition(1);
        transfer.setPower(0);
        intake.setPower(0);
        
        telemetry.addData("Action", "Shot complete");
        telemetry.update();
    }
    
    /**
     * Check if shooter is at target velocity
     * @return true if shooter is ready to shoot
     */
    private boolean isShooterReady() {
        return shooterEnabled && shooter.getVelocity() >= SHOOTER_TARGET_VELOCITY;
    }
    
    /**
     * Sleep while continuing to update shooter control
     * @param milliseconds Time to sleep in milliseconds
     */
    private void sleepWithShooter(long milliseconds) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < milliseconds) {
            updateShooter();
            sleep(50);
        }
    }
    
    // ==================== ALLIANCE SELECTION ====================
    
    /**
     * Select alliance color before start
     * Press X for Blue Alliance, B for Red Alliance
     */
    private void selectAlliance() {
        telemetry.addData("===== ALLIANCE SELECTION =====", "");
        telemetry.addData("Press X", "Blue Alliance");
        telemetry.addData("Press B", "Red Alliance");
        telemetry.addData("", "");
        telemetry.addData("Current Selection", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
        
        // Wait for alliance selection
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.x) {
                isRedAlliance = false;
                allianceMultiplier = -1;
                telemetry.addData("===== ALLIANCE SELECTION =====", "");
                telemetry.addData("Press X", "Blue Alliance");
                telemetry.addData("Press B", "Red Alliance");
                telemetry.addData("", "");
                telemetry.addData("Current Selection", "BLUE");
                telemetry.update();
                sleep(200);  // Debounce
            } else if (gamepad1.b) {
                isRedAlliance = true;
                allianceMultiplier = 1;
                telemetry.addData("===== ALLIANCE SELECTION =====", "");
                telemetry.addData("Press X", "Blue Alliance");
                telemetry.addData("Press B", "Red Alliance");
                telemetry.addData("", "");
                telemetry.addData("Current Selection", "RED");
                telemetry.update();
                sleep(200);  // Debounce
            }
        }
        
        telemetry.addData("Alliance Selected", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Path will be", isRedAlliance ? "Normal" : "Mirrored");
        telemetry.update();
    }
}

