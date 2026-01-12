package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - IMU Turn 90° (Fixed)", group = "ISDrive")
public class IMUTurn90Easy extends LinearOpMode {

    // === YOUR FOUR DRIVE MOTORS ===
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // === IMU ===
    private IMU imu;

    @Override
    public void runOpMode() {

        // ==== CONNECT EVERYTHING ====
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Reverse left side so forward = forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Connect IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // TELL IMU HOW YOUR CONTROL HUB IS MOUNTED
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        telemetry.addData("STATUS", "All motors + IMU connected!");
        telemetry.addData("INSTRUCTIONS", "Keep robot STILL → Press INIT → Wait for green light → Press PLAY");
        telemetry.update();

        // Reset yaw to 0 when you press INIT
        imu.resetYaw();

        waitForStart();

        telemetry.addData("STARTING", "Yaw = %.2f°", getHeading());
        telemetry.update();
        sleep(500);

        // TURN LEFT 90°
        turnToHeading(90);
        sleep(1000);

        // TURN RIGHT 90° (back to 0°)
        turnToHeading(0);

        telemetry.addData("ALL DONE", "Perfect IMU turns!");
        telemetry.update();
    }

    // ★★★★★ TURN TO EXACT HEADING (WITH LIVE TELEMETRY) ★★★★★
    private void turnToHeading(double targetDegrees) {

        telemetry.addData("TARGET", "%.1f°", targetDegrees);
        telemetry.update();

        while (opModeIsActive()) {
            double current = getHeading();
            double error = targetDegrees - current;

            // Always turn the shortest way
            if (error > 180) error -= 360;
            if (error <= -180) error += 360;

            telemetry.addData("Current Heading", "%.2f°", current);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Direction", error > 0 ? "LEFT" : "RIGHT");
            telemetry.update();

            if (Math.abs(error) <= 1.0) {  // within 1 degree = done!
                stopDriving();
                telemetry.addData("ARRIVED", "At %.2f°", getHeading());
                telemetry.update();
                sleep(500);
                break;
            }

            // Turn power (positive = right turn)
            double power = error > 0 ? 0.4 : -0.4;
            power = Math.max(0.25, Math.min(0.6, Math.abs(power)));

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);
        }
    }

    // ★★★★★ HELPER: GET CURRENT HEADING ★★★★★
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // ★★★★★ STOP ALL MOTORS ★★★★★
    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}