package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.35)
            .forwardZeroPowerAcceleration(-43.46051981916104)
            .lateralZeroPowerAcceleration(-66.40053118205549)
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.0, 0.1, 0.02))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.0735, 0.001729  ))
            .centripetalScaling(0)

//            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0.0, 0.01, 0.04))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.0, 0.1, 0.02))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0.0, 0.001, 0.6, 0.035))
//            .centripetalScaling(0.004)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorName("frontLeftDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(59.231774749101454)
            .yVelocity(47.89688270075987)
            ;

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.003947267 / 2f)
            .strafeTicksToInches(0.001952106)
            .turnTicksToInches(0.001980) // 0.001973747
            .leftPodY(6.5)
            .rightPodY(-6.5)
            .strafePodX(-5.8125)
            .leftEncoder_HardwareMapName("frontLeftDrive")
            .rightEncoder_HardwareMapName("indexer")
            .strafeEncoder_HardwareMapName("backLeftDrive")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.DOWN))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}