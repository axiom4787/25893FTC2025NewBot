package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.ForwardVelocityTuner;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)
            .forwardZeroPowerAcceleration(-33.71561842790841)
            .lateralZeroPowerAcceleration(-58.92532697943539)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.02, 0.025))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5,0,0.05,0.025))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.002,0,0.00002,0.6,0.1))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.035, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009,0.0,0.0001,0.6,0.02))
            .centripetalScaling(0.0008)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightDrive")
            .rightRearMotorName("backRightDrive")
            .leftRearMotorName("backLeftDrive")
            .leftFrontMotorName("frontLeftDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.27665265901823)
            .yVelocity(52.91157784048967);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(1.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.1, 0.7);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}