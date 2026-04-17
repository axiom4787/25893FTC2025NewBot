package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class DriveMotors {
    public final MotorEx frontLeft;
    public final MotorEx frontRight;
    public final MotorEx backLeft;
    public final MotorEx backRight;

    public DriveMotors(HardwareMap hwMap, String fl, String fr, String bl, String br) {
        frontLeft = new MotorEx(hwMap, fl, Motor.GoBILDA.RPM_312);
        frontRight = new MotorEx(hwMap, fr, Motor.GoBILDA.RPM_312);
        backLeft = new MotorEx(hwMap, bl, Motor.GoBILDA.RPM_312);
        backRight = new MotorEx(hwMap, br, Motor.GoBILDA.RPM_312);

        double cachingTol = 0.01;
        frontLeft.setCachingTolerance(cachingTol);
        frontRight.setCachingTolerance(cachingTol);
        backLeft.setCachingTolerance(cachingTol);
        backRight.setCachingTolerance(cachingTol);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setInverted(false);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        backRight.setInverted(true);
    }
}
