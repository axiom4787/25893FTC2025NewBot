package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class TestCachingMotors extends LinearOpMode {
    @Override
    public void runOpMode() {
        MotorEx motor = new MotorEx(hardwareMap, "frontLeftDrive", Motor.GoBILDA.RPM_312);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setCachingTolerance(0.01);

        waitForStart();

    }
}
