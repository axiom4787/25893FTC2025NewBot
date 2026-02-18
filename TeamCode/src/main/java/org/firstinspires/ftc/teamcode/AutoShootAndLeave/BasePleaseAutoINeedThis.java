package org.firstinspires.ftc.teamcode.AutoShootAndLeave;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Boilerplate.LimeLightCalculator;
import org.firstinspires.ftc.teamcode.Boilerplate.ThePlantRobotOpMode;

public class BasePleaseAutoINeedThis extends ThePlantRobotOpMode {
    private double shooterPower = 0.0;

    final double GUIDE_DOWN = 0.75;
    final double GUIDE_UP = 0.25;

    public enum Alliance { RED, BLUE, }
    public Alliance alliance = Alliance.BLUE;
    public boolean isBlue() { return alliance == Alliance.BLUE; }
    public boolean isRed() { return alliance == Alliance.RED; }

    @Override public void opModeInit() {}
    @Override public void opModeRunLoop() {}

    @Override public void opModeRunOnce() {
        setStatus("Starting auto");

        linearActuator.setPosition(GUIDE_DOWN);
        sleep(1000);

        setStatus("Aiming");
        drive(-0.4, 0.0, 0.0);
        double start = runtime.milliseconds();
        while (runtime.milliseconds() - start < 1000) { // 1 second
            autoTurret();
            autoLinearActuator();
            autoShooter();
            shooter.setPower(shooterPower);
        }

        setStatus("Shooting");
        drive(0.0, 0.0, 0.0); // Stop
        intake.setPower(1.0);
        sleep(5000);

        setStatus("Leaving");
        drive(0.0, isBlue() ? -0.67 : 0.67, 0.0);
        sleep(1000);

        setStatus("Finished");
        intake.setPower(0.0);
        shooter.setPower(0.0);
        drive(0, 0, 0);
    }

    private void setStatus(String status) {
        telemetry.addData("Status", status);
        telemetry.update();
    }

    double autoLinearActuatorValue = 0f;

    LimeLightCalculator LLC = new LimeLightCalculator(hardwareMap);

    private void autoLinearActuator() {
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

    double autoTurretValue = 0f;
    private void autoTurret() {
        LLResult target = LLC.getTarget();
        telemetry.addLine("AUTO TURRET");


        if (target != null) {
            autoTurretValue = LLC.calculateTurret(target);

            telemetry.addLine("Can see target!");
        } else {
            autoTurretValue /= 2f;
        }

        setTurretServosPower(autoTurretValue);
        telemetry.addData("Turret Direction", String.valueOf(autoTurretValue));

    }

    private void autoShooter() {
        LLResult target = LLC.getTarget();
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

    private void setTurretServosPower(double position) {
        // turretRight.setPosition(turretRight.getPosition() + position);
        // turretLeft.setPosition(turretRight.getPosition() + position);
    }
}