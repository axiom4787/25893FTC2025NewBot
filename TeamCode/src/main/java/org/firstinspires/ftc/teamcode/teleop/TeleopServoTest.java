package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Indexer Angle Test (Micro Mode)", group = "Test")
public class TeleopServoTest extends LinearOpMode {

    private Servo indexer;

    // GoBilda 2000-series Angular Servo range
    private static final double SERVO_MAX_DEGREES = 1800.0;
    private static final double SERVO_OFFSET = 238;

    // Micro-step size (in degrees)
    private static final double MICRO_STEP = 1.0;

    // Current commanded angle
    private double currentAngleDeg = 0;

    // Mode flag
    private boolean microMode = false;

    // Convert degrees to a 0–1 servo position
    private double degToPos(double degrees) {
        return Math.max(0, Math.min(1, degrees / SERVO_MAX_DEGREES));
    }

    // Utility: clamp angle to valid range
    private double clampAngle(double degrees) {
        return Math.max(0, Math.min(SERVO_MAX_DEGREES, degrees));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        indexer = hardwareMap.get(Servo.class, "indexer");

        telemetry.addLine("Indexer Angle Tester with Micro-Adjustment");
        telemetry.addLine("LB = Toggle Micro Mode");
        telemetry.addLine("Preset Mode Buttons: A/B/X/Y + Dpad Up/Down");
        telemetry.addLine("Micro Mode: Dpad Left/Right = -1° / +1°");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------------------------
            // MODE SWITCHING (Left Bumper)
            // ---------------------------
            if (gamepad1.left_bumper) {
                microMode = !microMode;
                sleep(250); // debounce
            }

            if (!microMode) {
                //
                // ---------------------------
                // PRESET ANGLE MODE
                // ---------------------------
                //

                if (gamepad1.a) currentAngleDeg = 0 + SERVO_OFFSET; // Intake 0 | 0
                if (gamepad1.b) currentAngleDeg = 194 + SERVO_OFFSET; // Shoot 0 | 180
                if (gamepad1.x) currentAngleDeg = 138 + SERVO_OFFSET; // Intake 1 | 120
                if (gamepad1.y) currentAngleDeg = 334 + SERVO_OFFSET; // Shoot 1 | 300
                if (gamepad1.dpad_up) currentAngleDeg = 271 + SERVO_OFFSET; // Intake 2 | 240
                if (gamepad1.dpad_down) currentAngleDeg = 466 + SERVO_OFFSET; // Shoot 2 | 460

            } else {
                //
                // ---------------------------
                // MICRO-ADJUST MODE
                // ---------------------------
                //

                if (gamepad1.dpad_left)   currentAngleDeg -= MICRO_STEP;
                if (gamepad1.dpad_right)  currentAngleDeg += MICRO_STEP;

                // Small delay to avoid too-fast stepping
                if (gamepad1.dpad_left || gamepad1.dpad_right) {
                    sleep(40);
                }
            }

            // Clamp angle within valid 0–300° range
            currentAngleDeg = clampAngle(currentAngleDeg);

            // Apply to servo
            indexer.setPosition(degToPos(currentAngleDeg));

            // Telemetry
            telemetry.addData("Mode", microMode ? "MICRO-ADJUST" : "PRESETS");
            telemetry.addData("Angle (deg)", currentAngleDeg - SERVO_OFFSET);
            telemetry.addData("Servo Pos (0–1)", indexer.getPosition());
            telemetry.update();
        }
    }
}