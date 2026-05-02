package org.firstinspires.ftc.teamcode.testOpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Context;

@TeleOp(name = "test vision pose")
public class TestTheRelocalization extends LinearOpMode {
    @Override
    public void runOpMode() {
        Context.init(hardwareMap);

        Vision vision = new Vision();

        waitForStart();

        while (opModeIsActive()) {
            Pose visionPose = vision.getPose();

            double x = visionPose.getX();
            double y = visionPose.getY();
            double h = visionPose.getHeading();

            telemetry.addLine("vision coords");
            telemetry.addData("vision x", x);
            telemetry.addData("vision y", y);
            telemetry.addData("vision h", h);
            telemetry.update();
        }
    }
}
