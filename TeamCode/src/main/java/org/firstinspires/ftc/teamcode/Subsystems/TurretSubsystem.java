package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Boilerplate.Config;

public class TurretSubsystem {
    Config config = new Config();
    CRServo leftTurretServo, rightTurretServo;
    AnalogInput servoEncoder;
    public final double MAX_SERVO_ANGLE = 100; // TODO: Adjust

    public TurretSubsystem(HardwareMap hardwareMap) {
        config.init(hardwareMap);

        leftTurretServo = config.turretServoLeft;
        rightTurretServo = config.turretServoRight;
        servoEncoder = config.axonServoEncoder;
    }

    public void setTurretPower(double power) {
        leftTurretServo.setPower(power);
        rightTurretServo.setPower(power);
    }

    public void setTurretPowerConstrained(double power) {
        // I am unsure of whether this code actually works
        if (getTurretAngle() > MAX_SERVO_ANGLE && power > 0) return;
        if (getTurretAngle() < -MAX_SERVO_ANGLE && power < 0) return;
        setTurretPower(power);
    }

    // Check out https://github.com/The-Robotics-Catalyst-Foundation/FIRST-Opensource/blob/main/FTC/RTPAxon/RTPAxon.java
    // for more possibly advantageous code
    public double getTurretAngle() {
        // TODO: Confirm if this works or if it's all a hoax
        return servoEncoder.getVoltage() / 3.3;
    }
}
