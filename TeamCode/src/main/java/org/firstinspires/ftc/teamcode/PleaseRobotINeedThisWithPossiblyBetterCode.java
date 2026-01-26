package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Boilerplate.ThePlantRobotOpMode;

@TeleOp(name="not comp ready code", group="Linear OpMode")
//@Disabled
public class PleaseRobotINeedThisWithPossiblyBetterCode extends ThePlantRobotOpMode {
    private boolean useHuskyLensForAim = true;
    private boolean driveInFieldRelative = true;
    private double lastHuskyLensPower = Shooter.SHOOT_POWER;

    private static class Hood {
        public static final double DOWN_POSITION = 0.75;
        public static final double UP_POSITION = 0.25;

        public static double clampPosition(double pos) {
            if (pos > Hood.DOWN_POSITION) pos = Hood.DOWN_POSITION;
            if (pos < Hood.UP_POSITION) pos = Hood.UP_POSITION;
            return pos;
        }

        public enum State {
            OFF,
            DOWN, // Lower hood pos -> close shooting
            UP,   // Raise hood pos -> far shooting
            AUTO, // Use HuskyLens to aim
        }
    }
    private static class Shooter {
        public static double OFF_POWER = 0.0;
        public static double SHOOT_POWER = 0.8;
        public static double REVERSE_POWER = -0.5;
        public enum State {
            OFF,
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
        public static double RIGHT_POWER = 1.0;
        public static double LEFT_POWER = -1.0;

        public enum State {
            OFF,
            AUTO,  // Use HuskyLens
            LEFT,
            RIGHT,
        }
    }

    private Hood.State hoodState = Hood.State.DOWN;
    private Shooter.State shooterState = Shooter.State.OFF;
    private Intake.State intakeState = Intake.State.OFF;
    private Turret.State turretState = Turret.State.AUTO;

    HuskyLens.Block target = null;

    @Override public void opModeInit() {}

    @Override public void opModeRunOnce() {
        gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Override public void opModeRunLoop() {
        robotControls();

        huskyLensSystem();

        driveSystem();
        intakeSystem();
        turretSystem();
        linearActuatorSystem();
        shooterSystem();
    }

    private void robotControls() {
        // Misc controls
        // A -> Reset gyro
        // B -> Toggle manual turret aim
        // X -> Toggle field relative
        if (gamepad1.aWasPressed()) {
            imu.resetYaw();
            gamepad1.rumble(100);
        }

        if (gamepad1.bWasPressed()) {
            useHuskyLensForAim = !useHuskyLensForAim;
            gamepad1.rumble(300);
        }

        if (gamepad1.xWasPressed()) {
            driveInFieldRelative = !driveInFieldRelative;
            gamepad1.rumble(600);
        }

        // Hood
        // If HuskyLens aiming is enabled:
        // - Use HuskyLens to determine linear actuator position
        // Otherwise:
        // - dpad down -> Move linear actuator to CLOSE shooting position
        // - dpad up   -> Move linear actuator to FAR shooting position
        if (useHuskyLensForAim) {
            hoodState = Hood.State.AUTO;
        } else {
            if (gamepad1.dpadUpWasPressed())         hoodState = Hood.State.UP;
            else if (gamepad1.dpadDownWasPressed())  hoodState = Hood.State.DOWN;
            else                                     hoodState = Hood.State.OFF;
        }

        // Turret
        // If HuskyLens aiming is enabled:
        // - Use HuskyLens to aim turret
        // Otherwise:
        // - dpad left  -> turn turret left
        // - dpad right -> turn turret right
        if (useHuskyLensForAim) {
            turretState = Turret.State.AUTO;
        } else {
            if (gamepad1.dpad_left)       turretState = Turret.State.LEFT;
            else if (gamepad1.dpad_right) turretState = Turret.State.RIGHT;
            else                          turretState = Turret.State.OFF;
        }

        // Shooter
        // If HuskyLens aiming enabled:
        // - Use HuskyLens to determine shooter speed
        // Otherwise:
        // - Right bumper -> Reverse (Also: reverse intake)
        // - Left trigger -> Shoot

        // It might be advantageous to map left bumper to reverse shooter instead of right bumper
        // so that they can be independently reversed
        if (gamepad1.left_trigger > 0) {
            if (useHuskyLensForAim) shooterState = Shooter.State.AUTO;
            else                    shooterState = Shooter.State.SHOOT;
        } else if (gamepad1.left_bumper) {
            shooterState = Shooter.State.REVERSE;
        } else {
            shooterState = Shooter.State.OFF;
        }

        // Intake
        // Right trigger -> Intake
        // Right bumper  -> Reverse intake (Also: reverse shooter)
        if (gamepad1.right_trigger != 0) intakeState = Intake.State.INTAKE;
        else if (gamepad1.right_bumper)  intakeState = Intake.State.REVERSE;
        else                             intakeState = Intake.State.OFF;

        // Log to driver station
        telemetry.addData("Hood state", hoodState.name());
        telemetry.addData("Shooter state", shooterState.name());
        telemetry.addData("Turret state", turretState.name());
        telemetry.addData("Intake state", intakeState.name());

        telemetry.addData("Driving mode", driveInFieldRelative ? "Field relative" : "Robot relative");
        telemetry.addData("Shooter aim control mode", useHuskyLensForAim ? "Auto" : "Manual");
    }

    private void huskyLensSystem() {
        if (!useHuskyLensForAim) return;

        target = getTargetBlock();
    }

    private void linearActuatorSystem() {
        switch (hoodState) {
            case AUTO:
                if (target == null) break;

                double actuatorPosition = linearActuator.getPosition();

                actuatorPosition -= (target.y - 110f) / 160f * 0.025;
                actuatorPosition = Hood.clampPosition(actuatorPosition);
                linearActuator.setPosition(actuatorPosition);
                break;
            case DOWN:
                linearActuator.setPosition(Hood.DOWN_POSITION);
//                hoodState = HOOD.STATE.OFF;
                break;
            case UP:
                linearActuator.setPosition(Hood.UP_POSITION);
//                hoodState = HOOD.STATE.OFF;
                break;
            case OFF:
                break;
        }

        telemetry.addData("Linear actuator position", "%4.2f", linearActuator.getPosition());
    }

    private void turretSystem() {
        switch (turretState) {
            case AUTO:
                if (target == null) {
                    setTurretServosPower(Turret.OFF_POWER);
                    break;
                }

                setTurretServosPower(- ( ( target.x / 160f ) - 1 ) * 0.5 );
                telemetry.addData("HuskyLens target height", target.height);

                break;
            case LEFT:
                setTurretServosPower(Turret.LEFT_POWER);
                break;
            case RIGHT:
                setTurretServosPower(Turret.RIGHT_POWER);
                break;
            case OFF:
                setTurretServosPower(Turret.OFF_POWER);
        }
    }

    private void shooterSystem() {
        double shooterPower = lastHuskyLensPower;
        switch (shooterState) {
            case SHOOT:
                shooterPower = Shooter.SHOOT_POWER;
                break;
            case REVERSE:
                shooterPower = Shooter.REVERSE_POWER;
                break;
            case AUTO:
                if (target == null) break;

                shooterPower = 0.8 - Math.max(0f, target.height - 30f) / 350f;
                lastHuskyLensPower = shooterPower;
                break;
            case OFF:
                shooterPower = Shooter.OFF_POWER;
        }

        shooter.setPower(shooterPower);
        telemetry.addData("Shooter power", "%.2f", shooterPower);
    }

    private void intakeSystem() {
        double intakePower = 0f;
        switch (intakeState) {
            case INTAKE:
                intakePower = gamepad1.right_trigger;
                break;
            case REVERSE:
                intakePower = Intake.REVERSE_POWER;
                break;
            case OFF:
                intakePower = Intake.OFF_POWER;
                break;
        }

        intake.setPower(intakePower);
        telemetry.addData("Intake power", "%.2f", intakePower);
    }

    private void driveSystem() {
        double forward = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
        double right   =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;

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

    private void setTurretServosPower(double position) {
        turretRight.setPower(position);
        turretLeft.setPower(position);
    }
}