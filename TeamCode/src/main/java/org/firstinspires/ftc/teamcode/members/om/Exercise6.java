package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Om - Exercise 6", group="Exercises")
public class Exercise6 extends LinearOpMode {

    public class ProgrammingBoard2 {
        private TouchSensor touchSensor;

        public ProgrammingBoard2(TouchSensor sensor) {
            this.touchSensor = sensor;
        }

        public boolean isTouchSensorReleased() {
            return !touchSensor.isPressed();
        }

    }

    private ProgrammingBoard2 programmingBoard;
    private TouchSensor hdwTouchSensor;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        hdwTouchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        programmingBoard = new ProgrammingBoard2(hdwTouchSensor);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean isReleased = programmingBoard.isTouchSensorReleased();

            String statusMessage;
            if (isReleased) {
                statusMessage = "Not Pressed";
            } else {
                statusMessage = "Pressed";
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Touch sensor status", statusMessage);
            telemetry.update();
        }
    }
}