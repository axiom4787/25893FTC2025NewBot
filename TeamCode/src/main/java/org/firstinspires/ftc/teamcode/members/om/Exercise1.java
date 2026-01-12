package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This OpMode appears in the Autonomous section of the Driver Station.
 * The "@Disabled" annotation has been removed to make it appear.
 */
@Autonomous(name="Om - Exercise 1", group="Exercises")
//@Disabled // Comment out this line to make the OpMode appear on the Driver Station
public class Exercise1 extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        telemetry.addData("Hello", "Om");
        telemetry.update();

        while (opModeIsActive()) {

        }
    }
}

