package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.limelightvision.LLStatus;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResultTypes;



@Autonomous(name = "Week0BackAuto", group = "Competition")
public class Week0BackAuto extends LinearOpMode {

    // Drivetrain motors 
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;

    // Subsystem motors 
    private DcMotorEx intake = null;
    private DcMotorEx transfer = null;
    private DcMotorEx shooter = null;

    // Servos 
    private Servo blocker = null;
    private Servo slidingDoor = null;
    private Servo kicker = null;

    // Sensors 
    private RevColorSensorV3 colorSensor = null;
    private GoBildaPinpointDriver odo = null;
    private static Limelight3A limelight = null;

    // Servo positions 
    private static final double BLOCKER_BLOCK_POSITION = 0.78;
    private static final double BLOCKER_UNBLOCK_POSITION = 0.632;
    private static final double SLIDING_DOOR_MIDDLE_POSITION = 1.0;
    private static final double SLIDING_DOOR_END_POSITION = 0.0;
    private static final double KICKER_UNKICK_POSITION = 0.65;
    private static final double KICKER_KICK_POSITION = 0.95;

    // Drivetrain constants
    private static final double MOTOR_PPR = 7.0;
    private static final double GEAR_RATIO = 12.7;
    private static final double WHEEL_DIAMETER_MM = 86.0;
    private static final double COUNTS_PER_MOTOR_REV = MOTOR_PPR * GEAR_RATIO;
    private static final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;
    private static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE_MM;
    
    // Conversion constant
    private static final double MM_TO_INCHES = 0.0393701;

    // Shooter settings 
    private static double TARGET_SHOOTER_VELOCITY = 1300.0;
    private boolean SHOOTING = false;
    private double kp = 0.15;
    private double kf = 0.045;

    // Ball detection enums 
    public enum BallColor {
        PURPLE,
        GREEN,
        NO_BALL
    }
    
    public enum ShotSeq {
        Setup,
        Fire,
        Idle
    }

    // Ball detection constants 
    private static final double MAX_DETECTION_DISTANCE_CM = 8.0;
    private static final int PURPLE_R = 71;
    private static final int PURPLE_G = 111;
    private static final int PURPLE_B = 130;
    private static final int GREEN_R = 57;
    private static final int GREEN_G = 145;
    private static final int GREEN_B = 128;

    // Sorting state variables 
    private boolean kFull = false;
    private boolean wFull = false;
    private boolean searching = true;
    private int ballNum = 0;
    private String currentMotif = null;
    private ShotSeq seq = ShotSeq.Idle;
    private long servoTimer = 0;

    // Alliance selection
    private boolean isRedAlliance = true;
    private int allianceMultiplier = 1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Initialize sorting system
        seq = ShotSeq.Idle;
        ballNum = 0;
        kFull = false;
        wFull = false;
        searching = true;

        telemetry.addData("Status", "Initialized. Ready to run!");
        telemetry.update();

        // Alliance selection
        selectAlliance();

        waitForStart();

        if (opModeIsActive()) {
            
            // ===== AUTONOMOUS SEQUENCE =====
            
            // Reset Pinpoint heading
            odo.resetPosAndIMU();
            
            currentMotif = "GPP";//getMotif();
            
            telemetry.addData("Motif Detected", currentMotif);
            telemetry.update();
            sleep(500);
            
            // Shoot preload
            TARGET_SHOOTER_VELOCITY = 1825;
            startShooter();
            turnToImu(157 * allianceMultiplier);
            waitForShooterReady();
            shoot(1800, 1.5);
            
            // drive to third row
            driveV2(1, 8); 
            turnToImu(-90 * allianceMultiplier);
            setIntake(true);
            driveV2(0.5, -10); // -3000mm (backward)
            driveV2(1, 12); // 4000mm
            setIntake(false);
            
            // drive to shoot position (2nd set)
            TARGET_SHOOTER_VELOCITY = 1500;
            turnToImu(180 * allianceMultiplier);
            driveV2(1, 20); // 5500mm
            turnToImu(140 * allianceMultiplier);
            sortAndShoot(3);
            
            //drive to second row
            turnToImu(-90 * allianceMultiplier);
            strafeV2(1, 18);
            turnToImu(-90 * allianceMultiplier);
            setIntake(true);
            driveV2(.5, -9);
            setIntake(false);
            
            
            driveV2(1, 9);
            strafeV2(1, -18);
            turnToImu(145 * allianceMultiplier);
            sortAndShoot(3);
            
            blocker.setPosition(BLOCKER_UNBLOCK_POSITION);
            kToMid();
            
            sleep(2000);

            //stopShooter();
            
            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    // ==================== HARDWARE INITIALIZATION ====================
    
    private void initializeHardware() {
        // Initialize drivetrain motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // Set motor directions for mecanum drive
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set drivetrain to brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize motors without encoder for V2 methods
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize subsystem motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Set motor directions for subsystems
        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // Configure shooter
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize servos
        blocker = hardwareMap.get(Servo.class, "blocker");
        slidingDoor = hardwareMap.get(Servo.class, "sliding_door");
        kicker = hardwareMap.get(Servo.class, "kicker");

        // Set initial servo positions
        blocker.setPosition(BLOCKER_BLOCK_POSITION);
        slidingDoor.setPosition(SLIDING_DOOR_END_POSITION);
        kicker.setPosition(KICKER_KICK_POSITION);

        // Initialize sensors
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        // Initialize GoBilda Pinpoint odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.resetPosAndIMU();
        odo.setOffsets(60, 35, DistanceUnit.MM);
        odo.setEncoderResolution(10.45, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        limelight.start();
        limelight.pipelineSwitch(0);
        
        // Reset encoders for V2 methods
        resetEncoders();
    }

    // ==================== PATHING METHODS ====================
    
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

    public void turnToImu(double targetHeading) {
        double roboHeading, error;
    
        do {
            roboHeading = getHeading();
            error = roboHeading - targetHeading;
            
            // Normalize error to be within -180 to 180 degrees
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
    
            double p = 0.0075, kf = 0.0;
    
            if (error < 0) {
                kf = -0.18;
            } else if (error > 0) {
                kf = 0.18;
            }
    
            frontRight.setPower((p * error + kf));
            frontLeft.setPower(-(p * error + kf));
            backRight.setPower((p * error + kf));
            backLeft.setPower(-(p * error + kf));
    
            telemetry.addData("error", error);
            telemetry.addData("heading", roboHeading);
            telemetry.addData("power", frontLeft.getPower());
            telemetry.update();
        } while (opModeIsActive() && !(error < 0.5 && error > -0.5));
        
        stopDrive();
    }
    
    
    public void driveV2(double power, double targetY) {
        resetEncoders();
        //odo.recalibrateIMU();
    
        double average = (frontRight.getCurrentPosition() + backRight.getCurrentPosition() + 
                         backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4;
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
    
            average = (frontRight.getCurrentPosition() + backRight.getCurrentPosition() + 
                      backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4;
            roboY = (average) * (4 * 3.1415 / 1120);
    
            roboHeading = getHeading();
    
            double error = roboHeading - targetHeading;
            // Normalize error to -180 to 180 degrees
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            
            double kp = (0.000);
    
            double a = error * kp;
            double errorD = targetY - roboY;
            double ep = 0.09;
    
            if (targetY > roboY) {
                kf = 0.12;
            } else {
                kf = -0.12;
            }
    
            frontRight.setPower((ep * errorD) * power - a + kf);
            backRight.setPower((ep * errorD) * power - a + kf);
            frontLeft.setPower((ep * errorD) * power + a + kf);
            backLeft.setPower((ep * errorD) * power + a + kf);
            
            updateShooter();
            
            telemetry.addData("CurrentHeading> ", roboHeading);
            telemetry.addData("TargetHeading> ", targetHeading);
            telemetry.addData("CurrentPos> ", roboY);
            telemetry.addData("TargetPos> ", targetY);
            telemetry.addData("average> ", average);
            telemetry.addData("fl power> ", frontLeft.getPower());
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
            // Normalize error to -180 to 180 degrees
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            
            double kp = 0;//(0.0118 / 0.3);
    
            double a = error * kp;
            double errorD = targetY - roboY;
            double ep = 0.061;
    
            if (targetY > roboY) {
                kf = 0.3;
            } else {
                kf = -0.3;
            }
    
            frontRight.setPower((ep * errorD) * power - a + kf);
            backRight.setPower((ep * errorD) * -power - a - kf);
            frontLeft.setPower((ep * errorD) * -power + a - kf);
            backLeft.setPower((ep * errorD) * power + a + kf);
            
            updateShooter();
            
            telemetry.addData("Heading", roboHeading);
            telemetry.addData("robo", roboY);
            telemetry.addData("Targ", targetY);
            telemetry.addData("average", average);
            telemetry.addData("flPower", frontLeft.getPower());
            telemetry.addData("frPower", frontRight.getPower());
            telemetry.addData("blPower", backLeft.getPower());
            telemetry.addData("brPower", backRight.getPower());
            telemetry.update();
        }
    
        stopDrive();
    }
    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // ==================== CONTROL METHODS ====================

    /**
     * Start the shooter and begin spinning up
     */
    private void startShooter() {
        SHOOTING = true;
        // Spin up shooter for a moment
        for (int i = 0; i < 20; i++) {
            updateShooter();
            sleep(50);
        }
    }

    /**
     * Stop the shooter completely
     */
    private void stopShooter() {
        SHOOTING = false;
        shooter.setPower(0);
    }

    /**
     * Update shooter power - call continuously
     */
    private void updateShooter() {
        if (SHOOTING) {
            double currentShooterVelocity = -shooter.getVelocity();
            double error = TARGET_SHOOTER_VELOCITY - currentShooterVelocity;
            
            double feedforward = kf;
            if (error < -800) {
                feedforward = 0.9;
            } else if (error < 0) {
                feedforward = 0.6;
            } else {
                feedforward = 0;
            }
            
            shooter.setPower(error * kp + feedforward);
        } else {
            shooter.setPower(0);
        }
    }

    /**
     * Wait for shooter to reach target velocity
     */
    private void waitForShooterReady() {
        while (opModeIsActive() && SHOOTING) {
            updateShooter();
            double currentVelocity = -shooter.getVelocity();
            
            if (Math.abs(TARGET_SHOOTER_VELOCITY - currentVelocity) < 50) {
                break;
            }
            
            telemetry.addData("Shooter", "Spinning up...");
            telemetry.addData("Velocity", "%.0f / %.0f", currentVelocity, TARGET_SHOOTER_VELOCITY);
            telemetry.update();
            sleep(50);
        }
    }

    /**
     * Turn intake on or off
     */
    private void setIntake(boolean on) {
        if (on) {
            intake.setPower(-1.0);
            transfer.setPower(-0.85);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    /**
     * Run intake for specified duration (no sorting)
     */
    private void intakeForDuration(long durationMs) {
        long startTime = System.currentTimeMillis();
        setIntake(true);
        
        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < durationMs) {
            updateShooter();
            telemetry.addData("Intaking", "Time: %d ms", System.currentTimeMillis() - startTime);
            telemetry.update();
            sleep(20);
        }
        
        setIntake(false);
    }

    // ==================== BALL DETECTION & SORTING ====================

    /**
     * Detects the ball color based on RGB values and distance
     */
    public BallColor detectBallColor() {
        if (colorSensor == null) {
            return BallColor.NO_BALL;
        }

        double distance = colorSensor.getRawLightDetected();
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        
        // if (distance < 85 || green == 0) {
        //     return BallColor.NO_BALL;
        // }
        
        double ratio = (double) red / green;

        if (ratio > 0.55) {
            return BallColor.PURPLE;
        } else {
            return BallColor.GREEN;
        }
    }

    public static String getMotif() {
        int limelightid = 0;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int id = fr.getFiducialId();
            limelightid = id;
        }
        if (limelightid == 21) {
            return "GPP";
        } else if (limelightid == 22) {
            return "PGP";
        } else if (limelightid == 23) {
            return "PPG";
        } else {
            return "PPG";
        }
    }

    /**
     * Run the sorting state machine - call this continuously while sorting
     */
    public void updateSorting() {
        BallColor detectedBall = detectBallColor();
        double error = TARGET_SHOOTER_VELOCITY - (-shooter.getVelocity());

        switch (seq) {
            case Setup:
                transfer.setPower(-0.85);
                intake.setPower(-0.75);
                
                if (searching) {
                    if (cFull(detectedBall)) {
                        if (colorCorrect(detectedBall)) {
                            toFire();
                        } else {
                            if (!kFull) {
                                moveToK();
                                searching = false;
                                servoTimer = System.currentTimeMillis();
                            } else if (!wFull) {
                                moveToW();
                                searching = false;
                                servoTimer = System.currentTimeMillis();
                            } else {
                                toFire();
                            }
                        }
                    } else if (kFull) {
                        kToMid();
                        searching = false;
                        servoTimer = System.currentTimeMillis();
                    } else if (wFull) {
                        wToMid();
                        searching = false;
                        servoTimer = System.currentTimeMillis();
                    } else {
                        ballNum = 0;
                        toIdle();
                    }
                } else {
                    if ((System.currentTimeMillis() - servoTimer) > 1000) {
                        searching = true;
                    }
                }
                break;
                
            case Fire:
                if (Math.abs(error) < 50 && !searching) {
                    blocker.setPosition(BLOCKER_UNBLOCK_POSITION);
                    transfer.setPower(-1);
                    searching = true;
                }

                if (searching && Math.abs(error) > 75) {
                    blocker.setPosition(BLOCKER_BLOCK_POSITION);
                    seq = ShotSeq.Setup;
                    searching = false;
                    servoTimer = System.currentTimeMillis();
                    ballNum++;
                }
                break;
                
            case Idle:
                break;
        }
    }

    private void toSetup() {
        seq = ShotSeq.Setup;
        SHOOTING = true;
    }
    
    private void shoot(double TargetVelo, double seconds) {
        
        ElapsedTime timer = new ElapsedTime();
        double currentShooterVelocity = shooter.getVelocity();
        double error = TargetVelo - currentShooterVelocity;
        timer.reset();
        
        while (timer.seconds() < seconds) {
            if (error < -800) {
                kf = 0.9;
            } else if (error < 0) {
                kf = 0.6;
            } else {
                kf = 0;
            }
            transfer.setPower(-0.7);
            intake.setPower(-0.5);
            blocker.setPosition(BLOCKER_UNBLOCK_POSITION);
            shooter.setPower(error * kp + kf);
        }
        blocker.setPosition(BLOCKER_BLOCK_POSITION);
    }
        
    private void toFire() {
        seq = ShotSeq.Fire;
        searching = false;
        servoTimer = System.currentTimeMillis();
    }

    private void toIdle() {
        blocker.setPosition(BLOCKER_BLOCK_POSITION);
        kicker.setPosition(KICKER_KICK_POSITION);
        moveToW();
        SHOOTING = false;
        shooter.setPower(0);
        wFull = false;
        seq = ShotSeq.Idle;
    }

    public void moveToK() {
        slidingDoor.setPosition(SLIDING_DOOR_MIDDLE_POSITION);
        kicker.setPosition(KICKER_UNKICK_POSITION);
        kFull = true;
    }

    public void moveToW() {
        slidingDoor.setPosition(SLIDING_DOOR_END_POSITION);
        wFull = true;
    }

    public void kToMid() {
        kicker.setPosition(KICKER_KICK_POSITION);
        slidingDoor.setPosition(SLIDING_DOOR_END_POSITION);
        kFull = false;
    }

    public void wToMid() {
        kicker.setPosition(KICKER_UNKICK_POSITION);
        slidingDoor.setPosition(SLIDING_DOOR_MIDDLE_POSITION);
        wFull = false;
    }

    public static String getMotif(int limelightid) {
        if (limelightid == 21) {
            return "GPP";
        } else if (limelightid == 22) {
            return "PPG";
        } else if (limelightid == 23) {
            return "PGP";
        } else {
            return "GPP";
        }
    }

    public char getColor(BallColor color) {
        if (color == BallColor.PURPLE) {
            return 'P';
        } else if (color == BallColor.GREEN) {
            return 'G';
        } else {
            return 'E';
        }
    }

    public boolean colorCorrect(BallColor color) {
        if (ballNum > 2) {
            return true;
        }
        return currentMotif.charAt(ballNum) == getColor(color);
    }

    public boolean cFull(BallColor color) {
        return color == BallColor.PURPLE || color == BallColor.GREEN;
    }

    /**
     * Run sorting process until specified number of balls are sorted
     */
    public void sortAndShoot(int numBalls) {
        ballNum = 0;
        toSetup();
        
        while (opModeIsActive() && ballNum < numBalls) {
            updateShooter();
            updateSorting();
            
            telemetry.addData("Sorting State", seq);
            telemetry.addData("Ball Number", ballNum);
            telemetry.addData("Motif", currentMotif);
            telemetry.addData("Detected", detectBallColor());
            telemetry.update();
            
            sleep(20);
        }
        
        toIdle();
    }

    // ==================== ALLIANCE SELECTION ====================

    private void selectAlliance() {
        telemetry.addData("===== ALLIANCE SELECTION =====", "");
        telemetry.addData("Press X", "Blue Alliance");
        telemetry.addData("Press B", "Red Alliance");
        telemetry.addData("", "");
        telemetry.addData("Current Selection", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
        
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.x) {
                isRedAlliance = false;
                allianceMultiplier = -1;
                telemetry.addData("Current Selection", "BLUE");
                telemetry.update();
                sleep(200);
            } else if (gamepad1.b) {
                isRedAlliance = true;
                allianceMultiplier = 1;
                telemetry.addData("Current Selection", "RED");
                telemetry.update();
                sleep(200);
            }
        }
        
        telemetry.addData("Alliance Selected", isRedAlliance ? "RED" : "BLUE");
        telemetry.update();
    }
}