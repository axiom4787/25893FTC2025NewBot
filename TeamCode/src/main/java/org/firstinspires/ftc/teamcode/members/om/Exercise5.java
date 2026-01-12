package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Om - Exercise 5", group="Exercises")
public class Exercise5 extends OpMode {

    public class RobotLocation {
        private double x;
        private double y;
        private double angle;

        public RobotLocation() {
            this.x = 0.0;
            this.y = 0.0;
            this.angle = 0.0;
        }

        public double getAngle() {
            return this.angle;
        }
        public double getX() {
            return this.x;
        }

        public void changeX(double change) {
            this.x += change;
        }

        public void setX(double x) {
            this.x = x;
        }

        public double getY() {
            return this.y;
        }

        public void changeY(double change) {
            this.y += change;
        }

        public void setY(double y) {
            this.y = y;
        }
    }

    private RobotLocation robotLocation = new RobotLocation();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            robotLocation.changeX(-0.1);
        } else if (gamepad1.dpad_right) {
            robotLocation.changeX(0.1);
        }

        if (gamepad1.dpad_up) {
            robotLocation.changeY(0.1);
        } else if (gamepad1.dpad_down) {
            robotLocation.changeY(-0.1);
        }

        telemetry.addData("Robot X", robotLocation.getX());
        telemetry.addData("Robot Y", robotLocation.getY());
        telemetry.addData("Robot Angle", robotLocation.getAngle());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}