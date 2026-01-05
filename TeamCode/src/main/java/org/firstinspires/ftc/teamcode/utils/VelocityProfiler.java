package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityProfiler {

    //double value = 0;

    LinearOpMode opMode;

    double smoothingFactor;

    double currentValueLeftY = 0;
    double currentValueLeftX = 0;
    double currentValueRightX = 0;

    double currentValue = 0;


    public VelocityProfiler(LinearOpMode opMode,double smoothingFactor){
    this.opMode = opMode;
    this.smoothingFactor = smoothingFactor;
    }

    //ElapsedTime timer = new ElapsedTime();

    //double maxRate = 2.0; // maximum rate of change of power allowed per second


//    public void resetElapsedTime(){
//        timer.reset();
//    };

//    public double velocityProfileIncrement(double value){
//        if(timer.milliseconds() >= 200) {
//        this.value = value;
//        if (value - currentValue > 0){
//            currentValue = currentValue + 0.2;
//        } else if (value - currentValue == 0) {
//            currentValue = currentValue + 0;
//        } else if (value - currentValue < 0) {
//            currentValue = currentValue - 0.2;
//        }
//            resetElapsedTime();
//        }
//        return currentValue;
//    }



//    public double update(double targetValue) { // targetvalue is the value u wan ur motors to reach
//
//        // Time since last update (seconds)
//        double deltaTime = timer.seconds();
//        timer.reset();
//
//        // Maximum change allowed this update
//        double maxChange = maxRate * deltaTime;
//
//        // Difference between target and current value
//        double error = targetValue - currentValue;
//
//        // If we can reach the target this update, snap to it
//        if (Math.abs(error) <= maxChange) {
//            currentValue = targetValue;
//        }
//        // Otherwise, move toward the target at the allowed rate
//        else {
//            currentValue += Math.signum(error) * maxChange;
//        }
//
//        return currentValue;
//    }

    public double velocityProfileLeftY(){
        currentValueLeftY = currentValueLeftY + (-opMode.gamepad1.left_stick_y - currentValueLeftY) * smoothingFactor;
        return currentValueLeftY;
    }
    public double velocityProfileLeftX(){
        currentValueLeftX = currentValueLeftY + (-opMode.gamepad1.left_stick_x - currentValueLeftX) * smoothingFactor;
        return currentValueLeftX;
    }
    public double velocityProfileRightX(){
        currentValueRightX = currentValueRightX + (-opMode.gamepad1.right_stick_x - currentValueRightX) * smoothingFactor;
        return currentValueRightX;
    }

    double velocityProfileProportional(double target){
        currentValue = currentValue + (target - currentValue) * smoothingFactor;
        return currentValue;
    }
// double power = velocityProfileIncrement(gamepad1.left_stick_y)

}
