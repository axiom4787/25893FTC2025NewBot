package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Import the DcMotorEx class
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDFCoefficients; // Import for PIDF control
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.AprilTagWebcam;

import java.util.List;


@TeleOp(name = "week0teleop", group = "Competition")
//@Disabled
public class Week0teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // drive motors
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    //other motors
    DcMotorEx intake, shooter, transfer, turret;
    // limelight



    // IMU
    private IMU imu;
    // Servos
    Servo hold;
    Servo arm;

    // --- State Variables for Toggles ---
    boolean shoot = false;
    boolean climbHold = false;
    boolean openHold = false;


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- HARDWARE MAPPING ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter"); // Mapped as DcMotorEx to read encoder
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(53, 0.02, 3, 11);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        hold = hardwareMap.get(Servo.class, "hold");
        arm = hardwareMap.get(Servo.class, "kicker");
        telemetry.setMsTransmissionInterval(5);
        shooter.setPower(0);


        // --- MOTOR DIRECTION ---
        // drive motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // motor behaviors

        // drive motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // other motrs
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motor modes
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // other motor modes
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

        aprilTagWebcam.init(hardwareMap, telemetry);


        //create constants
        //max 1800
        double newTargetVelocity = 1600.0;


        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        while (!opModeIsActive()) {

            telemetry.addData("Status", "Initialized. Ready to run!");
            telemetry.update();

        }
        waitForStart();

        runtime.reset();

        intake.setPower(1);

        while (opModeIsActive()) {

            ///drive code
            double y = gamepad1.left_stick_y;  // Forward
            double x = -gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x;  // Rotate

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Optional: Normalize powers if exceeding 1.0
            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (max > 1.0) {
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            //end drive code

            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

            double distance;

            distance = 0;

            if(openHold==false) {
                if (id20 != null) {
                    distance = id20.ftcPose.range;
                    telemetry.addData( "Distance ", distance);
                }

                if (distance > 55 && distance < 70) {
                    newTargetVelocity = 1500 * (distance / 58.0);
                }

                else if (distance > 30 && distance < 45) {
                    newTargetVelocity = 1300 * (distance / 38.0);
                }
                else if (distance > 40 && distance < 55) {
                    newTargetVelocity = 1425 * (distance / 47.0);
                }

                else if (distance > 90 && distance < 110) {
                    newTargetVelocity = 1900 * (distance / 105.0);
                }




            }
            telemetry.addData( "Target velocity ", newTargetVelocity);
            telemetry.addData( "Distance ", distance);




            if ((gamepad2.a || gamepad1.a) )
            {

                //trying to fire
                if (shooter.getVelocity() >= newTargetVelocity)
                {
                    //good to shoot
                    shooter.setPower(0.55);
                    shoot = true;
                    //sleep(100);
                } else {
                    //need to spin up more
                    shoot = false;
                    shooter.setPower(1);
                }
            }
            else
            {
                // Stop the shooter when not spinning up
                shooter.setPower(0);
                shoot = false;
            }

            if(shooter.getVelocity() >= (newTargetVelocity - 100))
            {
                openHold = true;
            }
            else
            {
                openHold = false;
            }

            // shooting
            //|| (shoot && aligned && limelightid == targetId && gamepad2.b)
            if (openHold) {
                hold.setPosition(0);
                transfer.setPower(1);
                //sleep(50);
                //arm.setPosition(1);
            }
            else {
                hold.setPosition(0.82);
                //arm.setPosition(0);
            }

            //hold.setPosition(
            //    (((double)newTargetVelocity - shooter.getVelocity())/newTargetVelocity)*0.1+0.3);
//hold.setPosition(0.3);

            if ((gamepad2.x || gamepad1.x || (shoot && (gamepad2.b || gamepad1.b))) ) {
                transfer.setPower(1);
                intake.setPower(-1);

            } else {


                if (gamepad1.right_trigger > 0.1) {
                    intake.setPower(-1);
                    transfer.setPower(1);
                } else if (gamepad1.left_trigger > 0.1) {
                    intake.setPower(1);
                    transfer.setPower(-1);

                } else {
                    intake.setPower(0);
                    if (!openHold)
                        transfer.setPower(0);
                }
            }

            //climber
            if(gamepad1.dpad_up || gamepad2.dpad_up)
            {
                turret.setPower(1);
            }
            else if (gamepad1.dpad_down || gamepad2.dpad_down)
            {
                climbHold = true;
            }else if(gamepad1.dpad_left) {
                turret.setPower(-0.1);
            }    else
            {
                if(climbHold)
                {
                    turret.setPower(0.5);
                } else
                {
                    turret.setPower(0.0);
                }


            }
            
            // telemetry
            telemetry.addData("Status", "Initialized");
            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Actual Velocity", "%.2f", shooter.getVelocity());
            telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
            telemetry.addData("Open Hold", openHold);
            telemetry.update();

        }
    }
}
