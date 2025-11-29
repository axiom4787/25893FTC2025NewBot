package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
@Config
public class Outtake {
    private final MotorEx shooter;
    private double power;
    final double TICKS_PER_REV = 112.0;
    public static double p=0.0002,i=0,d=0,f=0.0003;
    public static int targetRPM;
    public double RPM;
    private final PIDController controller;
    private final double TPR = 28; // ticks per rotation
    public boolean shooterEnabled=false;

    public Outtake(HardwareMap hardwareMap)
    {
        shooter = new MotorEx(hardwareMap, "outtake");
        power = -1;
        controller = new PIDController(p,i,d);
        shooter.setInverted(true);


    }

    public void stop()
    {
        shooter.stopMotor();
    }

    public void run()
    {
        shooter.set(power);
    }

    public void setPower(double newPower)
    {
        power = newPower;
        shooter.set(power);
    }


    public void periodic(){

            controller.setPID(p, i, d);
            RPM = shooter.getVelocity() / TPR * 60;
            double pid =controller.calculate(RPM, targetRPM);
            double ff = targetRPM * f;
            power = pid + ff;

            power = clamp(power, 1.0, 0);
            shooter.set(power);

    }

    public void setTargetRPM(int target){
        targetRPM=target;
    }
    public double getTargetRpm(){
        return targetRPM;
    }
    public double getRPM(){
        return RPM;
    }

    public double getPower()
    {
        return power;
    }
    public double clamp (double val,double max, double min){
        return Math.max(min,Math.min(max,val) );
    }

    //TODO: Make algorithm to determine power needed based on distance
}