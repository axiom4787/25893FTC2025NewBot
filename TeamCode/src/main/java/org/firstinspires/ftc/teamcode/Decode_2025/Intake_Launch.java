package org.firstinspires.ftc.teamcode.Decode_2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Launch {
    /* Declare OpMode members.
     * gain access to methods in the calling OpMode.
     */
    private LinearOpMode myOp = null;
    // Default constructor
    public Intake_Launch(LinearOpMode opmode) {
        myOp = opmode;
    }

    private DcMotorEx launch = null;// 6000 rpm motor
    private DcMotor   arm    = null; // 312 rpm motor
    private Servo     intake = null; // intake motor controller
    private Servo     guide = null; // intake motor controller


    public void InitIL(){
        // Define and Initialize Motor.

        launch = myOp.hardwareMap.get(DcMotorEx.class, "launch");
        arm    = myOp.hardwareMap.get(DcMotor.class, "arm");
        // Define and Initialize Servo
        intake = myOp.hardwareMap.get(Servo.class, "intake");
        guide  = myOp.hardwareMap.get(Servo.class, "guide");
    }


}
