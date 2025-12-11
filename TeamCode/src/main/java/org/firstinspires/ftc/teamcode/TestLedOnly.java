package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FIXED: LED Cycle Test", group = "Test")
public class TestLedOnly extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo testLed = hardwareMap.get(Servo.class, "rgbLight");
        waitForStart();

        // Test GREEN
        testLed.setPosition(0.5);
        telemetry.addData("Status", "HOLDING GREEN (0.5)");
        telemetry.update();
        sleep(3000); // Hold for 3 full seconds

        // Test RED
        testLed.setPosition(0.25);
        telemetry.addData("Status", "HOLDING RED (0.25)");
        telemetry.update();
        sleep(3000); // Hold for 3 full seconds

        telemetry.addData("Status", "Test Complete");
        telemetry.update();
        sleep(2000);
    }
}