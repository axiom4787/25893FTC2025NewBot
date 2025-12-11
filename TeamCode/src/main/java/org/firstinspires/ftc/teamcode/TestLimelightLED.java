package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.AprilTagLedManager;

@TeleOp(name = "Test: Limelight LED", group = "Test")
public class TestLimelightLED extends LinearOpMode {
    private AprilTagLedManager ledManager;

    @Override
    public void runOpMode() {
        ledManager = new AprilTagLedManager(hardwareMap, "limelight", "rgbLight");

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ledManager.update();

            telemetry.addData("Testing", "Limelight LED Manager");
            telemetry.addData("Action", "Find an AprilTag. LED should turn GREEN in target zone.");
            telemetry.addLine("---");
            telemetry.update();
        }
    }
}