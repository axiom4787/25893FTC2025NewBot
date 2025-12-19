package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.qualcomm.robotcore.util.ElapsedTime;


public class Motors {
    // Sets up a variable used for the loop to shoot three artifacts - Nikola
    private int shootAll = 0;

    ElapsedTime time = new ElapsedTime();


    public DcMotor intake = null;
    public DcMotorEx flywheel = null;
    public DcMotor shooter = null;
    public Servo ballEjector = null;
    public Servo fidgetTech = null;
    PIDFController velocityController;
    //PID controller constants
    private double kP = 0.5;  // P = proportional
    private double kI = 0.05; // I = integral
    private double kD = 0;    // D = derivative
    private double kF = 0;    // F = feedForward


    public int spinPosition;
    public final double[] spinPositions = {
            0.03,0.06,0.1,0.14,0.17,0.21,0.25,0.28,0.32,0.36,
            0.4,0.43,0.47,0.51,0.55,0.58,0.62,0.66,0.7,0.73,
            0.77,0.8,0.83,0.87,0.91,0.94,0.98
    };

    public void initMotors(HardwareMap hardwareMap) {
        // maps the motors and servos using the hardware map when called - Jason
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        ballEjector = hardwareMap.get(Servo.class, "ballEjector");
        fidgetTech = hardwareMap.get(Servo.class, "spinIndexer");

        spinPosition = 13;
        flywheel = hardwareMap.get(DcMotorEx.class, "Shooter");
        velocityController = new PIDFController(kP, kI, kD, kF);
        velocityController.setSetPoint(0);

    }
    // velocity is in RPM
    public void setFlywheelVelocity(double velocity){
        velocityController.setSetPoint(velocity*(28.0/60.0));
    }
    public void update() {
        double currentVelocity = flywheel.getVelocity();
        double power = velocityController.calculate(currentVelocity); // Get power from PIDF
        flywheel.setPower(power);

    }
    //This public void adds a function to shoot all three artifacts - Nikola
    public void shootAllBalls (){
        // Code to shoot all three artifacts when △ is pressed.
            spinPosition = 1; shootAll = 0; while(shootAll < 3){
            spinPosition += 2;
            shooter.setPower(0.7);
            time.reset();
            fidgetTech.setPosition(spinPositions[spinPosition]);
            while(time.seconds() < 4 ){
                //just chill
            }
            ballEjector.setPosition(0.3);
            time.reset();
            while (time.seconds() < 1) {
                //you get to chill again
            }
            ballEjector.setPosition(0);

            shootAll += 1;
        }


    }
}