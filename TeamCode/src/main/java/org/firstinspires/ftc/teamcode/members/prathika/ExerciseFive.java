package org.firstinspires.ftc.teamcode.members.prathika;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ExerciseFive extends OpMode {

    RobotLocation robotLocation = new RobotLocation(0);

    @Override
    public void init() {
        robotLocation.setAngle(0);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            robotLocation.turn(0.1);
        } else if (gamepad1.b) {
            robotLocation.turn(-0.1);
        }

        telemetry.addData("Heading", robotLocation.getHeading());
        telemetry.addData("X", robotLocation.getClass());
        telemetry.addData("Y", robotLocation.getClass());
        telemetry.update();
    }


    public class RobotLocation {

        private double heading;

        public RobotLocation(double initialHeading) {
            this.heading = initialHeading;
        }

        public void setAngle(double angle) {
            this.heading = angle;
        }

        public void turn(double delta) {
            this.heading += delta;
        }

        public double getHeading() {
            return heading;
        }
    }

}