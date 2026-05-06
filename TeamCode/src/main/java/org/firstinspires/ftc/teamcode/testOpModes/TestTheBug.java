package org.firstinspires.ftc.teamcode.testOpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class TestTheBug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(70.75, 70.75, 0));

        waitForStart();

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));

        while (opModeIsActive()) {
            follower.update();

//            hubs.forEach(LynxModule::clearBulkCache);

            follower.setHeading(follower.getHeading());

            telemetry.addData("follower h radians", "%2.3f",  follower.getHeading());
            telemetry.update();
        }
    }
}
