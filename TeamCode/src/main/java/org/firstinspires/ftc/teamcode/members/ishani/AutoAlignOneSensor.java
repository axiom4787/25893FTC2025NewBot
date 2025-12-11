/*
How to use RIGHT NOW:

Mount your ONE sensor on the left OR right side of robot (pointing sideways)
Name it sideSensor in config
Put robot near backdrop → Press INIT → PLAY
Watch it automatically center itself perfectly for high basket!

 */
package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "IS - Auto Align to Backdrop", group = "ISSensor")
public class AutoAlignOneSensor extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DistanceSensor sideSensor;   //  ONE sensor mounted on the SIDE!

    // ———————— SETTINGS (tune these!) ————————
    private static final double TARGET_DISTANCE_INCHES = 8.0;   // how far from backdrop side wall
    private static final double STRAFE_SPEED = 0.35;            // speed when adjusting
    private static final double TOLERANCE_INCHES = 0.5;         // stop when within 0.5 inch

    @Override
    public void runOpMode() {

        // Connect motors
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Connect your ONE sensor — name it "sideSensor" in config!
        sideSensor = hardwareMap.get(DistanceSensor.class, "sideSensor");

        telemetry.addData("AUTO ALIGN READY", "1-sensor centering!");
        telemetry.addData("Target distance from wall", "%.1f inches", TARGET_DISTANCE_INCHES);
        telemetry.addData("Mount sensor on LEFT or RIGHT side → Press INIT → PLAY", "");
        telemetry.update();

        waitForStart();

        telemetry.addData("ALIGNING TO BACKDROP...", "Strafing until perfect...");
        telemetry.update();

        while (opModeIsActive()) {

            double distance = sideSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("Side distance", "%.1f inches", distance);
            telemetry.addData("Target", "%.1f ± %.1f inches", TARGET_DISTANCE_INCHES, TOLERANCE_INCHES);

            // ——— PERFECTLY CENTERED! ———
            if (Math.abs(distance - TARGET_DISTANCE_INCHES) <= TOLERANCE_INCHES) {
                stopDriving();
                telemetry.addData("PERFECT ALIGN!", "Centered at %.1f inches", distance);
                telemetry.update();
                sleep(3000);
                break;
            }

            // ——— TOO FAR FROM WALL → strafe toward it ———
            else if (distance > TARGET_DISTANCE_INCHES + TOLERANCE_INCHES) {
                // If sensor is on LEFT side → strafe RIGHT to get closer
                // If sensor is on RIGHT side → strafe LEFT
                strafeRight(STRAFE_SPEED);
                telemetry.addData("ADJUSTING", "Too far → strafing toward wall");
            }

            // ——— TOO CLOSE TO WALL → strafe away ———
            else if (distance < TARGET_DISTANCE_INCHES - TOLERANCE_INCHES) {
                strafeLeft(STRAFE_SPEED);
                telemetry.addData("ADJUSTING", "Too close → strafing away");
            }

            telemetry.update();
        }

        stopDriving();
        telemetry.addData("AUTO ALIGN COMPLETE", "Ready to score in high basket!");
        telemetry.update();
    }

    // ———————— STRAFE RIGHT ————————
    private void strafeRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    // ———————— STRAFE LEFT ————————
    private void strafeLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    // ———————— STOP ————————
    private void stopDriving() {
        frontLeft.setPower(0); frontRight.setPower(0);
        backLeft.setPower(0); backRight.setPower(0);
    }
}