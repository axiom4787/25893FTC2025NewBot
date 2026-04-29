package org.firstinspires.ftc.teamcode.testOpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Hardware;

@TeleOp(name = "test vision pose")
public class TestTheRelocalization extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware.init(hardwareMap);

        Vision vision = new Vision(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            vision.update();
//            Pose visionPose = vision.getRobotPose();

//            double x = visionPose.getX();
//            double y = visionPose.getY();
//            double h = visionPose.getHeading();

            telemetry.addLine("vision coords");
//            telemetry.addData("vision x", x);
//            telemetry.addData("vision y", y);
//            telemetry.addData("vision h", h);
            telemetry.update();
        }
    }
}
