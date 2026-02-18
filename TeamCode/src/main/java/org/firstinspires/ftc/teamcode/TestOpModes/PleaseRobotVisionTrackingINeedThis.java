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

package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.Boilerplate.LimeLightCalculator;
import org.firstinspires.ftc.teamcode.Boilerplate.PID;
import org.firstinspires.ftc.teamcode.Boilerplate.ThePlantRobotOpMode;

@TeleOp(name="Robot Vision Tracking & Tuning Please I need this", group="Linear OpMode")
public class PleaseRobotVisionTrackingINeedThis extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooter;
    private Servo turretLeft, turretRight;
    private Servo linearActuator;
    private HuskyLens huskyLens;
    private IMU imu;
    private double shooterPower = 0.0;
    private boolean useHuskyLensForAim = true;
    final double GUIDE_DOWN = 0.75;
    final double GUIDE_UP = 0.25;
    LimeLightCalculator LLC;
    PID hoodPID;
    double P = 4e-5f;
    double D = 1e-5f;

    @Override
    public void runOpMode() {
        LLC = new LimeLightCalculator(hardwareMap);
        initElectronics();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        linearActuator.setPosition(GUIDE_DOWN);

        while (opModeIsActive()) {
            hoodPID = new PID(P, 0, D, -1, 1);

            //LLC.hoodPID = hoodPID;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("P", P);
            telemetry.addData("P", D);

            if (gamepad1.aWasPressed()) {
                P = P + 1e-5;
            }
            if (gamepad1.bWasPressed()) {
                P = P - 1e-5;
            }

            if (gamepad1.xWasPressed()) {
                D = D + 1e-5;
            }
            if (gamepad1.yWasPressed()) {
                D = D - 1e-5;
            }

            autoTurret();
            autoLinearActuator();
            autoShooter();

            if (gamepad1.right_bumper) {
                shooterPower = -0.5;
            }
            if (gamepad1.left_trigger > 0) {
                // shooter.setPower(shooterPower);
            } else {
                if (intakeSpeed > 0.1) {
                    shooter.setPower(-intakeSpeed * 2);
                } else {
                    shooter.setPower(shooterPower / 1.5);
                }
            }

            telemetry.update();
        }
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

        shooter = config.shooter;
        turretLeft = config.turretServoLeft;
        turretRight = config.turretServoRight;
        linearActuator = config.linearActuator;
//        smartServo = config.smartServo;

        imu = config.imu;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void linearActuator() {
          if (gamepad1.dpad_down) {
              linearActuator.setPosition(GUIDE_DOWN);
          }
          if (gamepad1.dpad_up) {
              linearActuator.setPosition(GUIDE_UP);
          }
          telemetry.addData("Linear actuator position", "%4.2f", linearActuator.getPosition());
    }
    double autoLinearActuatorValue = 0f;
    private void autoLinearActuator() {
        Config config = new Config();
        LLResult target = LLC.getTarget();
        telemetry.addLine("AUTO ACTUATOR");

        double actuatorPosition = linearActuator.getPosition();
        if (target != null) {
            autoLinearActuatorValue = LLC.calculateHood(target);

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

    private void turret() {
        double turretRotationChange = 0;
        if (gamepad1.dpad_right) turretRotationChange = 1;
        if (gamepad1.dpad_left) turretRotationChange = -1;

//        setTurretServosPower(turretRotationChange + smartServo.getPosition()); // fix me

//        telemetry.addData("Turret rotation offset", "%4.2f", turretRotationChange);
    }

    double autoTurretValue = 0f;
    private void autoTurret() {
        Config config = new Config();
        LLResult target = LLC.getTarget();
        telemetry.addLine("AUTO TURRET");


        if (target != null) {
            autoTurretValue = LLC.calculateTurret(target);

            telemetry.addLine("Can see target!");
            telemetry.addData("Target TX:", LLC.log(target, LimeLightCalculator.LogWhat.TX));
            telemetry.addData("Target TY:", LLC.log(target, LimeLightCalculator.LogWhat.TY));
        } else {
            autoTurretValue /= 1.2f;
        }
        setTurretServosPower(autoTurretValue);
        telemetry.addData("Turret Direction", String.valueOf(autoTurretValue));



    }

    private void autoShooter() {
        Config config = new Config();
        LLResult target = LLC.getTarget();
        if (target == null) return;

        shooterPower = 0.7 - Math.max(0f, target.getTx() - 30f) / 350f;
    }
    private void shooter() {
        shooterPower = 0.7;
    }

    double intakeSpeed;

    private void setTurretServosPower(double position) {
        turretLeft.setPosition(turretLeft.getPosition() + position);
    }
}