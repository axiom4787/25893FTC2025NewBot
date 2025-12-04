package org.firstinspires.ftc.teamcode.members.saksham.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp
public class Exercise5 extends LinearOpMode {
    RobotLocation robotLocation = new RobotLocation(0);
    Logger logger = new Logger(Logger.LoggerMode.CRITICAL, telemetry);
    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) robotLocation.turn(0.1);
            if (gamepad1.b) robotLocation.turn(-0.1);

            if (gamepad1.dpad_left) robotLocation.changeX(-0.1);
            if (gamepad1.dpad_right) robotLocation.changeX(0.1);

            if (gamepad1.dpad_up) robotLocation.changeY(0.1);
            if (gamepad1.dpad_down) robotLocation.changeY(-0.1);

            logger.logData(Logger.LoggerMode.CRITICAL, "Angle", robotLocation.getAngle());
            logger.logData(Logger.LoggerMode.CRITICAL, "X", robotLocation.getX());
            logger.logData(Logger.LoggerMode.CRITICAL, "Y", robotLocation.getY());
            logger.update();
        }
    }
}
class RobotLocation {

    double angle;
    double x;
    double y;

    public RobotLocation(double angle) {
        this.angle = angle;
    }

    public double getHeading() {
        double angle = this.angle;
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public double getAngle() {
        return angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    @Override
    public String toString() {
        return "RobotLocation: angle (" + angle + ")";
    }

    public void turn(double angleChange) {
        angle += angleChange;
    }

    public void changeX(double change) {
        x += change;
    }

    public void changeY(double change) {
        y += change;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }
}
