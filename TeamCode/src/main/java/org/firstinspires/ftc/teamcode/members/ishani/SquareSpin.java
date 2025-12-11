package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IS - Square Spin", group = "ISDrive")
public class SquareSpin extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {

        // === CONNECT MOTORS ===
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // === CONNECT IMU ===
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addData("SQUARE SPIN READY", "Keep robot still → Press INIT → Wait green light");
        telemetry.addData("Then press PLAY →", "Robot will spin a perfect square forever!");
        telemetry.update();

        imu.resetYaw();  // makes current direction = 0°

        waitForStart();

        telemetry.addData("GO!", "Spinning perfect square...");
        telemetry.update();

        // Spin a square 10 times (change 10 to any number or remove for infinite)
        for (int i = 1; i <= 10; i++) {
            turnToHeading(i * 90);   // 90°, 180°, 270°, 360°=0°...
            telemetry.addData("Square", "Side %d of 10 complete!", i);
            telemetry.update();
        }

        stopDriving();
        telemetry.addData("SQUARE DONE", "Perfect 10 squares completed!");
        telemetry.update();
    }

    // ★★★★★ TURN TO EXACT HEADING (used for square) ★★★★★
    private void turnToHeading(double targetDegrees) {

        double current = getHeading();
        double error = targetDegrees - current;

        // Always shortest direction
        while (error >  180) error -= 360;
        while (error <= -180) error += 360;

        telemetry.addData("→ Turning to", "%.0f°", targetDegrees);
        telemetry.update();

        while (opModeIsActive() && Math.abs(error) > 1.0) {

            double power = error > 0 ? 0.45 : -0.45;  // left or right
            power = Math.max(0.3, Math.abs(power));   // minimum speed so it moves

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            current = getHeading();
            error = targetDegrees - current;
            while (error >  180) error -= 360;
            while (error <= -180) error += 360;

            telemetry.addData("Turning", error > 0 ? "LEFT ↺" : "RIGHT ↻");
            telemetry.addData("Current", "%.2f°", current);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.update();
        }

        stopDriving();
        telemetry.addData("✓ Reached", "%.2f°", getHeading());
        telemetry.update();
        sleep(300);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}