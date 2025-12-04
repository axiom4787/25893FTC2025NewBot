package org.firstinspires.ftc.teamcode.FRLib.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUW {
    private final IMU imu;

    public IMUW(HardwareMap hw, String name,
                RevHubOrientationOnRobot.LogoFacingDirection logo,
                RevHubOrientationOnRobot.UsbFacingDirection usb) {

        imu = hw.get(IMU.class, name);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }


    /*
    * Gyro measurements
    */

    public double getYaw() {
        return getYaw(AngleUnit.DEGREES);
    }

    public double getYaw(AngleUnit unit) {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return -angles.getYaw(unit);
    }

    public void resetYaw() {
        imu.resetYaw();
    }
}
