package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class Hardware {
    //singleton
    private static Hardware instance;

    public final DcMotorEx intake;
    public final Servo sorter;
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;
    // Odometry
    public final GoBildaPinpointDriver pinPointOdo;

    public final Limelight3A limelight;

    public final WebcamName logitech;

    public Hardware(HardwareMap hwMap){
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR); //rightback
        this.intake = hwMap.get(DcMotorEx.class, Specifications.INTAKE);
        this.sorter = hwMap.get(Servo.class, Specifications.SORTER);


        this.pinPointOdo = hwMap.get(GoBildaPinpointDriver.class, Specifications.PIN_POINT_ODOMETRY);

        this.logitech = hwMap.get(WebcamName.class, "Webcam 1");

        this.limelight = hwMap.get(Limelight3A.class, "limelight");
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }
}
