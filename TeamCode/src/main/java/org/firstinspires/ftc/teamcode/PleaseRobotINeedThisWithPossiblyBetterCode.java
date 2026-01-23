/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="not comp ready code", group="Linear OpMode")
@Disabled
public class PleaseRobotINeedThisWithPossiblyBetterCode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor intake, shooter;
    private CRServo turretLeft, turretRight;
    private Servo linearActuator;
    private HuskyLens huskyLens;
    private IMU imu;

    private double shooterPower = 0.0;
    private boolean useHuskyLensForAim = true;
    private boolean driveInFieldRelative = true;

    private static class HOOD_POSITION {
        public static final double DOWN = 0.75;
        public static final double UP = 0.25;

        public static double clamp(double pos) {
            if (pos > HOOD_POSITION.DOWN) pos = HOOD_POSITION.DOWN;
            if (pos < HOOD_POSITION.UP) pos = HOOD_POSITION.UP;
            return pos;
        }
    }

    private static class SHOOTER {
        public static double OFF_POWER = 0.0;
        public static double SHOOT_POWER = 0.8;
        public static double REVERSE_POWER = -0.5;
    }

    private static class INTAKE {
        public static double OFF_POWER = 0.0;
        public static double REVERSE_POWER = 0.0;
    }

    private static class TURRET {
        public static double OFF_POWER = 0.0;
        public static double RIGHT_POWER = 1.0;
        public static double LEFT_POWER = -1.0;
    }

    @Override
    public void runOpMode() {
        initElectronics();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        linearActuator.setPosition(HOOD_POSITION.DOWN);
        // Set initial linear actuator position so that the code knows where it is.
        // The linear actuator can't actually read its position, it reads
        // the last set position as its position

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            robotControlToggles();

            driveSystem();
            intakeSystem();
            turretSystem();
            linearActuatorSystem();
            shooterSystem();

            telemetry.update();
        }
    }

    private HuskyLens.Block getTargetBlock() {
        HuskyLens.Block target = null;
        HuskyLens.Block[] blocks = huskyLens.blocks();
//        telemetry.addData("Block count", blocks.length);
        double biggestBlockSize = 0f;
        for (HuskyLens.Block block : blocks) {
            if (block.height * block.width > biggestBlockSize) {
                target = block;
                biggestBlockSize = block.height * block.width;
            }
//            telemetry.addData("Block", block.toString());
        }

        return target;
    }

    public double[] getRealPos(HuskyLens.Block targetBlock, double REAL_WIDTH) {
        double HALFCAMERAWIDTH = 320f / 2f;
        double HALFCAMERAHEIGHT = 180f / 2f;

        // Should be how much you need to multiply the distance calculation by to be accurate
        // You can find by running `TuneGetRealPos` and finding how much you need to multiply by to get the actual value, then multiply whichever of these you are tuning by that value to get your new value. These are the default values that produce +- 0.5 in of accuracy
        double ZSCALAR = 333.333; // How much you need to multiply the z distance calculation by to be accurate
        double XSCALAR = 0.555; // Same as above but for x distance
        double YSCALAR = 0.555; // """

        double targetBlockScale = Math.max(targetBlock.width, targetBlock.height); // our tag could be rotated, so we will take the largest of these to prevent errors

        double zDistance = (REAL_WIDTH / targetBlockScale) * ZSCALAR; // distance away in inches

        double xDistance = ((targetBlock.x - HALFCAMERAWIDTH) / HALFCAMERAWIDTH) * zDistance * XSCALAR; // the distance from the center in inches locally
        double yDistance = ((HALFCAMERAHEIGHT - targetBlock.y) / HALFCAMERAHEIGHT) * zDistance * YSCALAR;

        return new double[] {xDistance, yDistance, zDistance}; // Local x, y, and z, all in inches relative to the view
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

    private void robotControlToggles() {
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

        telemetry.addData("Driving mode", driveInFieldRelative ? "Field relative" : "Robot relative");
        telemetry.addData("Shooter aim control mode", useHuskyLensForAim ? "Auto" : "Manual");
    }

    private void linearActuatorSystem() {
        // If HuskyLens aiming is enabled:
        // - Use HuskyLens to determine linear actuator position
        // Otherwise:
        // - dpad down -> Move linear actuator to CLOSE shooting position
        // - dpad up   -> Move linear actuator to FAR shooting position

        if (useHuskyLensForAim) {
            HuskyLens.Block target = getTargetBlock();
            if (target == null) return;

            double actuatorPosition = linearActuator.getPosition();

            actuatorPosition -= (target.y - 110f) / 160f * 0.025;
//            actuatorPosition = HOOD_POSITION.clamp(actuatorPosition);
            linearActuator.setPosition(actuatorPosition);

        } else {
            if (gamepad1.dpad_down) { linearActuator.setPosition(HOOD_POSITION.DOWN); }
            if (gamepad1.dpad_up) {   linearActuator.setPosition(HOOD_POSITION.UP);   }
        }

        telemetry.addData("Linear actuator position", "%4.2f", linearActuator.getPosition());
    }

    private void turretSystem() {
        if (useHuskyLensForAim) {
            HuskyLens.Block target = getTargetBlock();

            if (target != null) {
                setTurretServosPower(-((target.x / 160f) - 1) * 0.5);
                telemetry.addData("Size", target.height);
            } else {
                setTurretServosPower(TURRET.OFF_POWER);
            }

        } else {
            double turretRotationChange = 0;
            if (gamepad1.dpad_right) turretRotationChange = TURRET.RIGHT_POWER;
            if (gamepad1.dpad_left) turretRotationChange = TURRET.LEFT_POWER;

            setTurretServosPower(turretRotationChange);
        }
    }

    private void shooterSystem() {
        // If HuskyLens aiming enabled:
        // - Use HuskyLens to determine shooter speed
        // Otherwise:
        // - Right bumper -> Reverse (Also reverses intake)
        // - Left trigger -> Shoot

        // It might be advantageous to map left bumper to reverse shooter instead of right bumper
        // so that they can be independently reversed

        if (gamepad1.left_trigger == 0) {
            shooterPower = SHOOTER.OFF_POWER;

        } else if (gamepad1.right_bumper) {
            shooterPower = SHOOTER.REVERSE_POWER;

        } else if (useHuskyLensForAim) {
            HuskyLens.Block target = getTargetBlock();
            if (target == null) return;

            shooterPower = 0.8 - Math.max(0f, target.height - 30f) / 350f;
        } else {
            shooterPower = SHOOTER.SHOOT_POWER;
        }

        shooter.setPower(shooterPower);
        telemetry.addData("Shooter power", "%.2f", shooterPower);
    }

    private void intakeSystem() {
        // Right trigger -> Intake
        // Right bumper  -> Outtake (Also reverses shooter)

        double intakeSpeed = gamepad1.right_trigger;
        if (gamepad1.right_bumper) intakeSpeed = INTAKE.REVERSE_POWER;

        intake.setPower(intakeSpeed);
        telemetry.addData("Intake speed", "%.2f", intakeSpeed);
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