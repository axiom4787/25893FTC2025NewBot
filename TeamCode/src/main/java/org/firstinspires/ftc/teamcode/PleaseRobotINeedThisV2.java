package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Boilerplate.LimeLightCalculator;
import org.firstinspires.ftc.teamcode.Boilerplate.RTPAxon;
import org.firstinspires.ftc.teamcode.Boilerplate.ThePlantRobotOpMode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

@TeleOp(name="comp ready code", group="Linear OpMode")
//@Disabled
public class PleaseRobotINeedThisV2 extends ThePlantRobotOpMode {
    private boolean useLimeLightForAim = true;
    private boolean driveInFieldRelative = true;
    private boolean driveSlower = false;
    private double lastLimeLightVelocity = Shooter.SHOOT_VELOCITY;
    DcMotorEx goodShooter;

    private static class Hood {
        public static final double DOWN_POSITION = 0.75;
        public static final double MIDDLE_POSITION = 0.5;
        public static final double UP_POSITION = 0.25;

        public static double clampPosition(double pos) {
            if (pos > Hood.DOWN_POSITION) pos = Hood.DOWN_POSITION;
            if (pos < Hood.UP_POSITION) pos = Hood.UP_POSITION;
            return pos;
        }

        public enum State {
            OFF,
            DOWN, // Lower hood pos -> don't shoot you'll probably miss from anywhere
            MIDDLE, // Middle hood pos -> close shooting
            UP,   // Raise hood pos -> far shooting
            AUTO, // Use HuskyLens to aim
        }
    }
    private static class Shooter {
        public static double OFF_VELOCITY = 0;
        public static double SHOOT_VELOCITY = 1800;
        public static double REVERSE_VELOCITY = -500;
        public static double IDLE_VELOCITY = 1000;
        public enum State {
            OFF,
            IDLE,
            SHOOT,
            REVERSE,
            AUTO,
        }
    }
    private static class Intake {
        public static double OFF_POWER = 0.0;
        public static double REVERSE_POWER = -0.5;

        public enum State {
            OFF,
            INTAKE,
            REVERSE,
        }
    }
    private static class Turret {
        public static double OFF_POWER = 0.0;
        public static double RIGHT_POWER = -1.0;
        public static double LEFT_POWER = 1.0;

        public enum State {
            OFF,
            AUTO,  // Use HuskyLens
            LEFT,
            RIGHT,
        }
    }
    private static class Indexer {
        enum State { OFF, FORWARD, REVERSE, }
        public static final double OFF_POWER = 0.0;
        public static final double FORWARD_POWER = 0.4;
        public static final double REVERSE_POWER = -1.0;
    }

    private Hood.State hoodState = Hood.State.DOWN;
    private Shooter.State shooterState = Shooter.State.OFF;
    private Intake.State intakeState = Intake.State.OFF;
    private Turret.State turretState = Turret.State.AUTO;
    private Indexer.State indexerState = Indexer.State.OFF;

    RTPAxon smartServoController;

    LLResult target = null;

    @Override public void opModeInit() {
        LLC = new LimeLightCalculator(hardwareMap);
        smartServoController = new RTPAxon(config.turretServoLeft, config.axonServoEncoder, RTPAxon.Direction.REVERSE);
        smartServoController.setRtp(false);
        smartServoController.forceResetTotalRotation();

        goodShooter = (DcMotorEx) shooter;
        goodShooter.setVelocityPIDFCoefficients(600, 0, 0, 14.6);
        // i don't think F is supposed to be that big...
    }

    LimeLightCalculator LLC;

    @Override public void opModeRunOnce() {
        gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Override public void opModeRunLoop() {
        smartServoController.update();
        robotControls();

        huskyLensSystem();

        driveSystem();
        intakeSystem();
        turretSystem();
        linearActuatorSystem();
        shooterSystem();
        indexerSystem();
    }

    private void robotControls() {
        // Controls
        // A / cross    -> Reset gyro
        // B / circle   -> Toggle LimeLight/manual control modes
        // X / square   -> Toggle field relative
        // Y / triangle -> Hold to drive slower for better parking

        //                  | intake    | indexer   | shooter   |
        // -----------------+-----------+-----------+-----------+
        // Left trigger     | ---       | ---       | Shoot     |
        // Left bumper      | Reverse   | Reverse   | Reverse   |
        // Right trigger    | Forward   | Reverse   | ---       |
        // Right bumper     | Forward   | Forward   | ---       |

        if (gamepad1.aWasPressed()) {
            imu.resetYaw();
            gamepad1.rumble(100);
        }

        if (gamepad1.bWasPressed()) {
            useLimeLightForAim = !useLimeLightForAim;
            gamepad1.rumble(300);
        }

        if (gamepad1.xWasPressed()) {
            driveInFieldRelative = !driveInFieldRelative;
            gamepad1.rumble(600);
        }

        driveSlower = gamepad1.y;

        // Hood
        // If LimeLight aiming is enabled:
        // - Use LimeLight to determine linear actuator position
        // - dpad down -> Move linear actuator to CLOSE shooting position
        // - dpad up   -> Move linear actuator to FAR shooting position
        if (gamepad1.dpadUpWasPressed())         hoodState = Hood.State.UP;
        else if (gamepad1.dpadDownWasPressed())  hoodState = Hood.State.MIDDLE;
        else if (useLimeLightForAim)             hoodState = Hood.State.AUTO;
        else                                     hoodState = Hood.State.OFF;

        // Turret
        // If LimeLight aiming is enabled:
        // - Use LimeLight to aim turret
        // - dpad left  -> turn turret left
        // - dpad right -> turn turret right
        if (gamepad1.dpad_left)       turretState = Turret.State.LEFT;
        else if (gamepad1.dpad_right) turretState = Turret.State.RIGHT;
        else if (useLimeLightForAim)  turretState = Turret.State.AUTO;
        else                          turretState = Turret.State.OFF;

        // Shooter
        // If LimeLight aiming enabled:
        // - Use LimeLight to determine shooter speed
        // Otherwise:
        // - Right bumper -> Reverse
        // - Left trigger -> Shoot
        // - Left bumper  ->

        boolean isDriving = gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0;

        if (gamepad1.left_trigger > 0) {
            shooterState = useLimeLightForAim
                    ? Shooter.State.AUTO
                    : Shooter.State.SHOOT;
        } else if (gamepad1.left_bumper) {
            shooterState = Shooter.State.REVERSE;
        } else if (!isDriving) {
            // If not driving, idle
            shooterState = Shooter.State.IDLE;
        } else {
            // If driving, turn off shooter idling to give more power to drive
            shooterState = Shooter.State.OFF;
        }

        // Intake
        // Right trigger -> Intake
        // Right bumper  -> Intake WITH INDEXER -> Shoot
        // Left bumper   -> Reverse Intake, Indexer, and Shooter
        if (gamepad1.right_trigger != 0)    intakeState = Intake.State.INTAKE;
        else if (gamepad1.right_bumper)     intakeState = Intake.State.INTAKE;
        else if (gamepad1.left_bumper)      intakeState = Intake.State.REVERSE;
        else                                intakeState = Intake.State.OFF;

        // Indexer
        // Right trigger -> Intake + reverse Indexer -> Intake
        // Right bumper  -> Intake + Index -> Shoot
        // Left bumper   -> Reverse Intake, Indexer, and Shooter
        if (gamepad1.right_trigger != 0)    indexerState = Indexer.State.REVERSE;
        else if (gamepad1.right_bumper)     indexerState = Indexer.State.FORWARD;
        else if (gamepad1.left_bumper)      indexerState = Indexer.State.REVERSE;
        else                                indexerState = Indexer.State.OFF;

        // Log to driver station
//        telemetry.addData("Hood state   ", hoodState.name());
//        telemetry.addData("Shooter state", shooterState.name());
//        telemetry.addData("Turret state ", turretState.name());
//        telemetry.addData("Intake state ", intakeState.name());
//        telemetry.addData("Indexer state", indexerState.name());

        telemetry.addData("Driving mode", driveInFieldRelative ? "FIELD relative" : "ROBOT relative");
        telemetry.addData("Shooter aim control mode", useLimeLightForAim ? "LIMELIGHT" : "MANUAL");
    }


    private void huskyLensSystem() {
        if (!useLimeLightForAim) return;

        target = LLC.getTarget();
    }

    double autoLinearActuatorValue = 0.0;
    double actuatorPosition = Hood.DOWN_POSITION;
    private void linearActuatorSystem() {
        switch (hoodState) {
            case AUTO:
                LLResult target = LLC.getTarget();
                if (target == null) {
                    autoTurretValue /= 1.5;
                    break;
                }

                autoLinearActuatorValue = LLC.calculateHood(target);
                actuatorPosition -= autoLinearActuatorValue;
                linearActuator.setPosition(actuatorPosition);
                telemetry.addData("Hood pos", actuatorPosition);
                telemetry.addData("Hood value", autoLinearActuatorValue);
                break;
            case DOWN:
                linearActuator.setPosition(Hood.DOWN_POSITION);
                break;
            case MIDDLE:
                linearActuator.setPosition(Hood.MIDDLE_POSITION);
                break;
            case UP:
                linearActuator.setPosition(Hood.UP_POSITION);
                break;
            case OFF:
                break;
        }

        telemetry.addData("Linear actuator position", "%4.2f", linearActuator.getPosition());
    }

    double autoTurretValue = 0;
    private void turretSystem() {
        telemetry.addData("servo angle", getTurretAngle());
        switch (turretState) {
            case AUTO:
                LLResult target = LLC.getTarget();

                if (target != null) {
                    autoTurretValue = LLC.calculateTurret(target);
                } else {
                    autoTurretValue /= 1.2f;
                }
                setTurretPowerConstrained(autoTurretValue);

                break;
            case LEFT:
                setTurretPowerConstrained(Turret.LEFT_POWER);
                break;
            case RIGHT:
                setTurretPowerConstrained(Turret.RIGHT_POWER);
                break;
            case OFF:
                setTurretPower(Turret.OFF_POWER);
        }
    }

    private void shooterSystem() {
        double shooterVelocity = lastLimeLightVelocity;

        switch (shooterState) {
            case SHOOT:
                shooterVelocity = Shooter.SHOOT_VELOCITY;
                break;
            case REVERSE:
                shooterVelocity = Shooter.REVERSE_VELOCITY;
                break;
            case IDLE:
                shooterVelocity = Shooter.IDLE_VELOCITY;
                break;
            case AUTO:
                LLResult target = LLC.getTarget();
                if (target == null) break;

                telemetry.addLine("CAN SEE TARGET");
                List<LLResultTypes.FiducialResult> fidResults = target.getFiducialResults();
                Pose3D tagPose = fidResults.get(0).getTargetPoseCameraSpace();
                double x = tagPose.getPosition().x;
                double z = tagPose.getPosition().z;
                double dist = Math.hypot(x, z);
                telemetry.addData("target distance", dist);
                shooterVelocity = dist > 0.5 ? ( dist > 1.7 ? 2000 : (365.08107 * dist + 1216.35886)) : 1400;

                lastLimeLightVelocity = shooterVelocity;
                break;
            case OFF:
                shooterVelocity = Shooter.OFF_VELOCITY;
        }

        goodShooter.setVelocity(shooterVelocity);
        telemetry.addData("Shooter actual velocity", goodShooter.getVelocity());
        telemetry.addData("Shooter set velocity", shooterVelocity);
    }

    private void indexerSystem() {
        double indexerPower = Indexer.OFF_POWER;
        switch (indexerState) {
            case FORWARD:
                indexerPower = Indexer.FORWARD_POWER;
                break;
            case REVERSE:
                indexerPower = Indexer.REVERSE_POWER;
                break;
        }

        indexer.setPower(indexerPower);
    }

    private void intakeSystem() {
        double intakePower = 0f;
        switch (intakeState) {
            case INTAKE:
                intakePower = 0.8;
                break;
            case REVERSE:
                intakePower = Intake.REVERSE_POWER;
                break;
            case OFF:
                intakePower = Intake.OFF_POWER;
                break;
        }

        intake.setPower(intakePower);
        telemetry.addData("Intake power", intakePower);
    }

    private void driveSystem() {
        double forward = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
        double right   =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;

        if (driveSlower) {
            forward /= 2;
            right /= 2;
            rotate /= 2;
        } // slow it down for parking

        if (driveInFieldRelative) {
            driveFieldRelative(forward, right, rotate);
        } else {
            drive(forward, right, rotate);
        }
    }
    private void drive(double forward, double right, double rotate) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double frontRightPower = forward - right - rotate;
        double frontLeftPower  = forward + right + rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        // Normalize the values so no wheel power exceeds 100%
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

//        telemetry.addData("Front left/Right drive", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//        telemetry.addData("Back  left/Right drive", "%4.2f, %4.2f", backLeftPower, backRightPower);
    }
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert cartesian offset to polar coordinates
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

    public void setTurretPower(double power) {
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }

    public final double MAX_SERVO_ANGLE = 150;
    public void setTurretPowerConstrained(double power) {
        if (
            (getTurretAngle() > MAX_SERVO_ANGLE && power < 0) ||
            (getTurretAngle() < -MAX_SERVO_ANGLE && power > 0)
        ) {
            setTurretPower(0.0);
        } else {
            setTurretPower(power);
        }
    }

    // Check out https://github.com/The-Robotics-Catalyst-Foundation/FIRST-Opensource/blob/main/FTC/RTPAxon/RTPAxon.java
    // for more possibly advantageous code
    public double getTurretAngle() {
        return smartServoController.getTotalRotation();
    }
}