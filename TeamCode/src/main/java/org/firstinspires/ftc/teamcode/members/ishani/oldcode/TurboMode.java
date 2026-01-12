package org.firstinspires.ftc.teamcode.members.ishani.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IS TurboMode")
public class TurboMode extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    @Override //
    public void runOpMode() {

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right_motor");

        waitForStart();

        while (opModeIsActive()) {

            // 1. Get the forward/backward joystick value (-1 to +1)
            double forward = -gamepad1.left_stick_y;  //negative because pushing up give negative

            // 2. Check if A button is pressed
            double speedMultiplier;
            if (gamepad1.a) {
                speedMultiplier = 1.0;   //TURBO FULL PWR
            } else {
                speedMultiplier =
                        0.5;   // Slow mode
            }

            // 3. Multiply the joystick by our multiplier
            double finalPower = forward * speedMultiplier;

            // 4. Send that power to your motors (example for tank drive)
            // leftDrive.setPower(finalPower);
            // rightDrive.setPower(finalPower);

            // 5. Show the driver what mode we're in
            if (gamepad1.a) {
                telemetry.addData("Forward Speed", "100% (TURBO!)");
            } else {
                telemetry.addData("Forward Speed", "50% (Normal)");
            }
            telemetry.addData("Raw Joystick", forward); // optional, just to see
            telemetry.update();
        }
    }
}