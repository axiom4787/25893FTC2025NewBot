package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {
    public final DcMotor intakeMot, shootingMot;
    public final Servo indexer, lifter;
    public final GoBildaPinpointDriver pinpoint;
    public final IMU imu;
    public final Telemetry telemetry;
    public final RevColorSensorV3 sensorR, sensorL;
    public final WebcamName camera;
    /*
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    public final Position cameraPosition = new Position(DistanceUnit.INCH, 0.00, 1.13, 13.74, 0);
    public final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    public RobotHardware(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMot = hwMap.get(DcMotor.class, "intakeMot"); //C0
        shootingMot = hwMap.get(DcMotor.class, "shootingMot"); //C1

        indexer = hwMap.get(Servo.class, "indexer"); //C0
        lifter = hwMap.get(Servo.class, "lifter"); //C1

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint"); //I2C 1
        imu = hwMap.get(IMU.class, "imu");

        sensorL = hwMap.get(RevColorSensorV3.class, "sensorL"); //I2C 2
        sensorR = hwMap.get(RevColorSensorV3.class, "sensorR"); //I2C 3
        camera = hwMap.get(WebcamName.class, "Webcam 1");

        setMotorDirections();
    }

    private void setMotorDirections() {
        intakeMot.setDirection(DcMotor.Direction.REVERSE);
        shootingMot.setDirection(DcMotor.Direction.REVERSE);
        intakeMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootingMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}