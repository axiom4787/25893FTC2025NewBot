package org.firstinspires.ftc.teamcode.members.vishruth;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FRLib.robot.Robot;
import org.firstinspires.ftc.teamcode.FRLib.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Logger;
//@TeleOp
public class OdometryFun extends OurOpmode {
    //         *  Left of the center is a positive number, right of center is a negative number.
    FieldObject blueGoal = new FieldObject(new Pose2D(DistanceUnit.INCH,-138,138, AngleUnit.DEGREES,0));
    Logger logger = new Logger(telemetry);
    Robot robot ;
    GoBildaPinpointDriver pinpoint;

    @Override
    protected void Loop() {
        robot.drive.driveVectorField(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,0.5,robot.imu);

        if(gamepad1.aWasPressed()){
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        }

        if (gamepad1.bWasPressed()){
            faceFieldObject(blueGoal,pinpoint.getPosition());
        }


    }

    @Override
    protected void initialize() {
    robot = new Robot(this,logger, MecanumDrive.RobotName.KEVIN);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    }

    public void faceFieldObject(FieldObject object,Pose2D robotPosition) {
        double xOffset = object.position.getX(DistanceUnit.INCH) + robotPosition.getX(DistanceUnit.INCH);
        double yOffset = object.position.getY(DistanceUnit.INCH) - robotPosition.getY(DistanceUnit.INCH);

        double angle = Math.toDegrees(Math.atan2(yOffset,xOffset));
        robot.drive.turnDegreesPID(angle-robotPosition.getHeading(AngleUnit.DEGREES),0.2,1);
    }
}
