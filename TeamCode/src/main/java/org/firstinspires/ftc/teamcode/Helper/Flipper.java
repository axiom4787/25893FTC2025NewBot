package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flipper {
    private Servo flipper;
    private OpMode opMode;

    public static final double flipperStart = 1.0;
    public static final double flipperIncrement = -0.2222;

    public void init(HardwareMap hwMap) {
        flipper = hwMap.get(Servo.class, "flipper");
        //flipper.scaleRange(-flipperIncrement, flipperStart);
        this.opMode = opMode;
        resetFlipper();
    }

    public void setPosition(double position){
        flipper.setPosition(position);
    }

    public void resetFlipper() {
        setPosition(flipperStart);
    }

    public void turnFlipper() {

       setPosition( 1 + flipperIncrement*135/360); // 1/3 turn
       // setPosition( Math.max(getPosition() + flipperIncrement, -flipperIncrement/2) );
    }
    public double getPosition(){
        return flipper.getPosition();
    }

}
