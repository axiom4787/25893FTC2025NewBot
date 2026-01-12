package org.firstinspires.ftc.teamcode.members.bala;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveImu {

    private  IMU imu;

    private void init(HardwareMap HwMap) {
        imu = HwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD

        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double getheading (AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    }
}