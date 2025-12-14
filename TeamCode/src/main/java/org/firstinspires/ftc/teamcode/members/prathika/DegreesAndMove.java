package org.firstinspires.ftc.teamcode.members.prathika;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DegreesAndMove extends LinearOpMode {

    static final double     COUNTS_PER_DEGREE       = 11.06;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // 5203 Series Yellow -Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public void runOpMode(){


    }
    public void moveWithDegrees(double speed, double distance, double degrees){

    }
}
