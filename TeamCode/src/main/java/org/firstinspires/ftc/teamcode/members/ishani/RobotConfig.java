package org.firstinspires.ftc.teamcode.members.ishani;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * ALL OF YOUR ROBOT SETTINGS IN ONE PLACE!
 * Change these numbers → every OpMode + Robot class uses them automatically
 */
public final class RobotConfig {

    // ———————— CALIBRATION NUMBERS (from your 3 calibrators!) ————————
    public static final double FORWARD_TICKS_PER_INCH = 48.7;
    public static final double STRAFE_TICKS_PER_INCH  = 52.3;
    public static final double TICKS_FOR_90_DEGREES   = 528.0;

    // ———————— IMU ORIENTATION (how your Control Hub is mounted) ————————
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // ———————— DRIVE SPEEDS ————————
    public static final double DRIVE_SPEED = 0.6;
    public static final double TURN_SPEED  = 0.5;

    // ———————— DISTANCE SENSOR SETTINGS ————————
    public static final String DISTANCE_SENSOR_NAME = "distance";

    // Prevent anyone from making a copy of this class
    private RobotConfig() {}
}