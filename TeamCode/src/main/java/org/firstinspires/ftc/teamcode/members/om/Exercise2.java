package org.firstinspires.ftc.teamcode.members.om;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Om - Exercise 2", group="Exercises")
public class Exercise2 extends OpMode {
    @Override
    public void init() {
        String myName = "Om";
        telemetry.addData("Hello", myName);
        int grade = 100; // Placeholder grade
        telemetry.addData("Grade", grade);
    }

    @Override
    public void loop() {
    }
}

