package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherControl {

    private DcMotor launchMotor;
    private double ticksPerRotation;

    public void init(HardwareMap hardwareMap){
        launchMotor = hardwareMap.get(DcMotor.class, "shooter");
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRotation = launchMotor.getMotorType().getTicksPerRev();

    }

    public void launchBall(double power) {
        launchMotor.setPower(power);
    }

    public double getLaunchSpeed() {
        return launchMotor.getCurrentPosition() / ticksPerRotation;
    }
}
