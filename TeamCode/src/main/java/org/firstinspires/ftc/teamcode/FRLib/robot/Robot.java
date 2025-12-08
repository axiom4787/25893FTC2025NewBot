package org.firstinspires.ftc.teamcode.FRLib.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.FRLib.hardware.Distance2mW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.IMUW;
import org.firstinspires.ftc.teamcode.FRLib.hardware.TouchSensorW;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;

public class Robot {
    public final MecanumDrive drive;
    public final IMUW imu;
    public final TouchSensorW touchSensor;
    public final Distance2mW distanceSensor;

    public Robot(LinearOpMode opMode, Logger logger, MecanumDrive.RobotName robotName) {
        drive = new MecanumDrive(opMode, logger, robotName);
        imu = new IMUW(opMode.hardwareMap, "imu",
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        touchSensor = new TouchSensorW(opMode.hardwareMap, "sensor_touch");
        distanceSensor = new Distance2mW(opMode.hardwareMap, "distance_sensor");
    }
}
