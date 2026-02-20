package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.Boilerplate.RTPAxon;

public class TurretSubsystem {
    Config config = new Config();
    CRServo leftTurretServo, rightTurretServo;
    AnalogInput servoEncoder;
    public final double MAX_SERVO_ANGLE = 140;
    RTPAxon smartServoController;

    public TurretSubsystem(HardwareMap hardwareMap) {
        config.init(hardwareMap);

        leftTurretServo = config.turretServoLeft;
        rightTurretServo = config.turretServoRight;
        servoEncoder = config.axonServoEncoder;

        smartServoController = new RTPAxon(leftTurretServo, servoEncoder, RTPAxon.Direction.REVERSE);
        smartServoController.forceResetTotalRotation();
    }

    public void setTurretPower(double power) {
        leftTurretServo.setPower(power);
        rightTurretServo.setPower(power);
    }

    public void setTurretPowerConstrained(double power) {
        if (getTurretAngle() > MAX_SERVO_ANGLE && power > 0) return;
        if (getTurretAngle() < -MAX_SERVO_ANGLE && power < 0) return;
        setTurretPower(power);
    }

    // Check out https://github.com/The-Robotics-Catalyst-Foundation/FIRST-Opensource/blob/main/FTC/RTPAxon/RTPAxon.java
    // for more possibly advantageous code
    public double getTurretAngle() {
        return smartServoController.getTotalRotation();
    }
}
