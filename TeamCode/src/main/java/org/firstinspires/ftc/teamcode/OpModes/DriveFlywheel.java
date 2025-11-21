package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.MainThread;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PID.FlywheelPID;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;

import java.math.MathContext;

@TeleOp(name = "Red Side Drive")
public class DriveFlywheel extends OpMode {

    private double previousPid = 0.0;   // initialize PID contribution
    private double previousPower = 0.0; // for optional full motor ramp
    private boolean flywheelOn = false;

    private double targetRPM = 4000;

    private Limelight3A limelight;
    private IMU imu;
    private final double rpmStep = 50;
    private boolean upPressed = false;
    private boolean downPressed = false;

    private FlywheelPID pid;
    Drivetrain drive = new Drivetrain();
    Intake intake = new Intake();

    private ElapsedTime timer = new ElapsedTime();
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;

    public DcMotorEx SIntake;
    private boolean xWasPressed = false;
    boolean autoAim = false;
    boolean yWasPressed = false;
    boolean bPrev = false;
    boolean intakeOn = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelL");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "flywheelR");
        SIntake = hardwareMap.get(DcMotorEx.class, "intake2");

        leftFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        pid = new FlywheelPID(0.0005, 0, 0.004);

        previousPid = 0.0;
        previousPower = 0.0;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
    }



    @Override
    public void start() {
        timer.reset();
        limelight.start();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean input1 = gamepad1.left_bumper;
        boolean input3 = false;
        boolean input2 = gamepad1.x;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botpose = llResult.getBotpose_MT2();
        }
        telemetry.addData("Ta", llResult.getTa());
        telemetry.addData("Tx", llResult.getTx());
        telemetry.addData("Ty", llResult.getTy());

        if (gamepad1.x && !yWasPressed) {
            autoAim = !autoAim;    // toggle
        }
        yWasPressed = gamepad1.x;

        double tx = llResult.getTx();
        boolean hasTarget = (llResult.isValid());
        if (autoAim && hasTarget) {
            double kP = 0.01 ;
            turn = kP * tx;

            if (Math.abs(tx) < 1.0) {
                turn = 0;
            }
        }
        drive.driveRobotRelative(y,x,turn);
        intake.NonStationary(input1);


        if (input2 && !xWasPressed) {
            flywheelOn = !flywheelOn;
        }
        xWasPressed = input2;

        double kF = flywheelOn ? 0.7 : 0.0;

        double leftVel = leftFlywheel.getVelocity();
        double rightVel = rightFlywheel.getVelocity();
        double avgRPM = (leftVel + rightVel) / 2 / 28.0 * 60.0;

        double pidOut = pid.calculate(targetRPM, avgRPM, 0.02);


        double rampedPid = previousPid + Math.signum(pidOut - previousPid) * 0.01;
        rampedPid = Math.max(-1, Math.min(1, rampedPid));

        double power = kF + rampedPid;

        if (!flywheelOn) power = 0;

        double minusTRPM = 200;

        if (gamepad1.b && !bPrev) {

            if (!intakeOn) {
                // Trying to turn ON — only allow if below RPM limit
                if (avgRPM < targetRPM &&  avgRPM > (targetRPM - minusTRPM)) {
                    intakeOn = true;
                }
            } else {
                // Trying to turn OFF — always allowed
                intakeOn = false;
            }
        }
        gamepad1.b = bPrev;
        SIntake.setPower(intakeOn ? 1.0 : 0.0);

        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);

        previousPid = rampedPid;

        telemetry.addData("Flywheel", flywheelOn ? "spinning :)" : "not spinning :(");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", avgRPM);
        telemetry.addData("Power", power);
    }}
