package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class IMUTest extends LinearOpMode {

    private IMU imu;

    @Override
    public void runOpMode(){

        imu = hardwareMap.get(IMU.class,"imu");
        // tell code that imu represents the imu in the hub
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );
        imu.initialize(new IMU.Parameters(RevOrientation));

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addData("Pitch",imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addData("Roll",imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

}
