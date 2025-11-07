package org.firstinspires.ftc.teamcode.Helper;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Opmodes.AutoBasketBlue;

public class Kicker {

    private Servo kickerPos;
    private OpMode opMode;

    public static final double gateClose = 0.37;
    public static final double gateShoot = 0.21;
    public static final double gateIntake = 0.6;

    public static final String GATE_CLOSE = "GATE_CLOSE";
    public static final String GATE_SHOOT = "GATE_SHOOT";
    public static final String GATE_INTAKE = "GATE_INTAKE";

    private String gatePosition;

    public void init(HardwareMap hwMap) {
        kickerPos = hwMap.get(Servo.class, "kicker");
        this.opMode = opMode;
        setGatePosition(GATE_INTAKE);
    }
    public void setGatePosition(String gatePosition){
        this.gatePosition = gatePosition;

        if(gatePosition.equals(GATE_CLOSE)){
            kickerPos.setPosition(gateClose);
        } else if(gatePosition.equals(GATE_SHOOT)){
            kickerPos.setPosition(gateShoot);
        }  else if(gatePosition.equals(GATE_INTAKE)){
            kickerPos.setPosition(gateIntake);
        } else {
            return;
        }

    }

    public String getGatePosition(){
        return gatePosition;
    }

    public void setPosition(double position){
        kickerPos.setPosition(position);
    }

    public double getPosition(){
        return kickerPos.getPosition();
    }

}
