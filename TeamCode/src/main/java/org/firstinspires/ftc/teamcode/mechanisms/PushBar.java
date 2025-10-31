package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.List;
import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PushBar {
    private Servo leftBar;
    private Servo rightBar;

    public void init(HardwareMap hardwareMap) {
        leftBar = hardwareMap.get(Servo.class, "LeftServo");
        rightBar = hardwareMap.get(Servo.class, "RightServo");
    }

    public List<Double> getBarPosition() {
        List<Double> barPosition = new ArrayList<Double>();

        barPosition.add(leftBar.getPosition());
        barPosition.add(rightBar.getPosition());

        return barPosition;
    }
    public void pushBall(double leftBarPosition, double rightBarPosition) {
        leftBar.setDirection(Servo.Direction.FORWARD);
        rightBar.setDirection(Servo.Direction.REVERSE);
        leftBar.setPosition(leftBarPosition);
        rightBar.setPosition(rightBarPosition);
    }

    public void release(double leftBarPosition, double rightBarPosition) {
        leftBar.setDirection(Servo.Direction.REVERSE);
        rightBar.setDirection(Servo.Direction.FORWARD);
        leftBar.setPosition(leftBarPosition);
        rightBar.setPosition(rightBarPosition);
    }



}


