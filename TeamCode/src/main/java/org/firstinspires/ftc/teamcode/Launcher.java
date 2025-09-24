package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher
{
    private Servo agitator = null;
    private DcMotor flywheel = null;
    private DcMotorEx shooterIntake = null;

    public Launcher(HardwareMap hwMap)
    {
        agitator  = hwMap.get(Servo.class, "agitator");
        flywheel  = hwMap.get(DcMotor.class, "flywheel");
        shooterIntake  = hwMap.get(DcMotorEx.class, "shooterIntake");

        //Remember to set the power for both motors
        //leftFrontDriveWheel.setPower(0);  //Here is an example change the variable name

        //Remember to set the power for both motors
        //leftFrontDriveWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
