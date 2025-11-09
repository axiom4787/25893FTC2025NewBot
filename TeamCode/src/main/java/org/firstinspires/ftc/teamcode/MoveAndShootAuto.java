package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous OpMode that:
 * 1. Moves backward 70 inches
 * 2. Shoots 3 preloaded balls
 */
@Autonomous(name="Move and Shoot Auto", group="Autonomous")
public class MoveAndShootAuto extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    
    // Shooter/Intake motor
    private DcMotor intakeMotor = null;
    
    // Constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: goBILDA 312 RPM Yellow Jacket
    private static final double DRIVE_GEAR_REDUCTION = 1.0;       // No external gearing
    private static final double WHEEL_DIAMETER_INCHES = 4.0;      // For calculating distance
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * Math.PI);
    
    private static final double DRIVE_SPEED = 0.6;                // Speed for driving
    private static final double SHOOT_POWER = 1.0;                // Full power for shooting
    private static final double SHOOT_TIME = 0.5;                 // Time to shoot each ball (seconds)
    private static final double PAUSE_BETWEEN_SHOTS = 0.3;        // Pause between shots (seconds)
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        
        // Initialize hardware
        initHardware();
        
        // Display initialization status
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Action", "Move back 70 inches, then shoot 3 balls");
        telemetry.update();
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            
            // Step 1: Move backward 70 inches
            telemetry.addData("Step 1", "Moving backward 70 inches");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, -70, -70, 5.0);  // Negative for backward
            
            // Brief pause to stabilize
            sleep(500);
            
            // Step 2: Shoot 3 balls
            telemetry.addData("Step 2", "Shooting 3 balls");
            telemetry.update();
            
            for (int i = 1; i <= 3; i++) {
                telemetry.addData("Shooting ball", i);
                telemetry.update();
                
                // Shoot (run intake motor at full power)
                intakeMotor.setPower(SHOOT_POWER);
                sleep((long)(SHOOT_TIME * 1000));
                
                // Stop between shots
                intakeMotor.setPower(0);
                
                // Pause between shots (except after last shot)
                if (i < 3) {
                    sleep((long)(PAUSE_BETWEEN_SHOTS * 1000));
                }
            }
            
            // Step 3: Complete
            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
            
            // Keep motors stopped
            stopAllMotors();
        }
    }
    
    /**
     * Initialize all hardware
     */
    private void initHardware() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        
        // Set motor directions (same as your teleop)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set all motors to use encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set zero power behavior to brake for stability
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * Drive using encoders for precise movement
     * 
     * @param speed Speed to drive (0.0 to 1.0)
     * @param leftInches Distance for left side (negative = backward)
     * @param rightInches Distance for right side (negative = backward)
     * @param timeoutS Maximum time allowed for the movement
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Calculate new target positions
            newLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            // Set target positions
            frontLeftDrive.setTargetPosition(newLeftTarget);
            backLeftDrive.setTargetPosition(newLeftTarget);
            frontRightDrive.setTargetPosition(newRightTarget);
            backRightDrive.setTargetPosition(newRightTarget);

            // Turn on RUN_TO_POSITION mode
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // Keep looping while we are still active, and there is time left, and both motors are running
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && 
                    backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // Display progress
                telemetry.addData("Target", "Running to %7d : %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Current Position", "FL:%7d FR:%7d BL:%7d BR:%7d",
                        frontLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(),
                        backLeftDrive.getCurrentPosition(),
                        backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            stopAllMotors();

            // Turn off RUN_TO_POSITION mode
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    /**
     * Stop all drive motors
     */
    private void stopAllMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
