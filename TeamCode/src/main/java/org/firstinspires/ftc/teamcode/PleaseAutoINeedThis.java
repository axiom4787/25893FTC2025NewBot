package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Auto BLUE: shoot & move", group="Basic Auto")
public class PleaseAutoINeedThis extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intake, shooter;
    private CRServo turretLeft, turretRight;
    private Servo linearActuator;
    private HuskyLens huskyLens;
    private IMU imu;
    private double shooterPower = 0.0;
    private boolean useHuskyLensForAim = true;

    final double GUIDE_DOWN = 0.75;
    final double GUIDE_UP = 0.25;

    @Override
    public void runOpMode() { // TODO: Guys its untested please test
        initElectronics();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Starting Auto");
        linearActuator.setPosition(GUIDE_DOWN);
        sleep(1000);
        driveFieldRelative(-0.5, 0.0, 0.0);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) { // 1000 milliseconds
            telemetry.addData("Status", "Aiming");
            autoTurret();
            autoLinearActuator();
            autoShooter();
            shooter.setPower(shooterPower);
        }
        telemetry.addData("Status", "Shooting");
        driveFieldRelative(0.0, 0.0, 0.0); // Stop
        intake.setPower(1.0);
        sleep(5000);
        telemetry.addData("Status", "Finished");
        intake.setPower(0.0);
        shooter.setPower(0.0);
    }

    private void autoControlToggles() {
        if (gamepad1.bWasPressed()) {
            useHuskyLensForAim = !useHuskyLensForAim;
            gamepad1.rumble(300);
        }

        telemetry.addData("Shooter aim control mode", useHuskyLensForAim ? "Auto" : "Manual");
    }
    private void initElectronics() {
        Config config = new Config();
        config.init(hardwareMap);

        frontLeftDrive = config.frontLeftDrive;
        frontRightDrive = config.frontRightDrive;
        backLeftDrive = config.backLeftDrive;
        backRightDrive = config.backRightDrive;
        intake = config.intake;
        shooter = config.shooter;
        turretLeft = config.turretServoLeft;
        turretRight = config.turretServoRight;
        linearActuator = config.linearActuator;
        huskyLens = config.huskyLens;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        imu = config.imu;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private HuskyLens.Block getTargetBlock() {
        HuskyLens.Block target = null;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        double size = 0f;
        for (int i = 0; i < blocks.length; i++) {
            HuskyLens.Block block = blocks[i];
            if (block.height * block.width > size) {
                target = block;
                size = block.height * block.width;
            }
        }

        return target;
    }

    double autoLinearActuatorValue = 0f;
    private void autoLinearActuator() {
        HuskyLens.Block target = getTargetBlock();
        telemetry.addLine("AUTO ACTUATOR");

        double actuatorPosition = linearActuator.getPosition();
        if (target != null) {
            double base = (target.y - 110f) / 160f * 0.035f;
            autoLinearActuatorValue = Math.signum(base) * Math.pow(Math.abs(base), 1.8f) * 100f;


            actuatorPosition -= autoLinearActuatorValue; // Modified to make it more extreme as you get farther away, hopefully making vision smarter
            //if (actuatorPosition > GUIDE_DOWN) actuatorPosition = GUIDE_DOWN;
            //if (actuatorPosition < GUIDE_UP) actuatorPosition = GUIDE_UP;
            linearActuator.setPosition(actuatorPosition);
        } else {
            autoTurretValue /= 1.5f;
        }
        telemetry.addData("Actuator Position", actuatorPosition);
        telemetry.addData("Actuator Change", autoLinearActuatorValue);
    }

    double autoTurretValue = 0f;
    private void autoTurret() {
        HuskyLens.Block target = getTargetBlock();
        telemetry.addLine("AUTO TURRET");


        if (target != null) {
            double base = -((target.x / 160f) - 1f) * 0.5f;
            autoTurretValue = Math.signum(base) * Math.pow(Math.abs(base), 1.4f) * 2.5f; // Similar to -((target.x / 160f) - 1) * 0.5, but make it increase more the farther away it is instead of linearly

            telemetry.addLine("Can see target!");
            telemetry.addData("Size", target.height);
        } else {
            autoTurretValue /= 1.2f;
        }
        setTurretServosPower(autoTurretValue);
        telemetry.addData("Turret Direction", String.valueOf(autoTurretValue));



    }

    private void autoShooter() {
        HuskyLens.Block target = getTargetBlock();
        if (target == null) return;

        shooterPower = 1.0; // Make it 1 to make sure it hits all the shots
    }

    private void drive(double forward, double right, double rotate) {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double frontRightPower = forward - right - rotate;
        double frontLeftPower  = forward + right + rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.abs(frontLeftPower);
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        telemetry.addData("Front left/Right drive", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right drive", "%4.2f, %4.2f", backLeftPower, backRightPower);
    }
    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }
    private void setTurretServosPower(double position) {
        turretRight.setPower(position);
        turretLeft.setPower(position);
    }
}