package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.Boilerplate.Config;
import org.firstinspires.ftc.teamcode.Boilerplate.RTPAxon;

public class TurretSubsystem {
    CRServo leftTurretServo, rightTurretServo;
    AnalogInput servoEncoder;
    public static final double MAX_SERVO_ANGLE = 140;
    RTPAxon smartServoController;

    public TurretSubsystem(
            CRServo turretServoLeft, CRServo turretServoRight,
            AnalogInput axonServoEncoder
    ) {
        this.leftTurretServo = turretServoLeft;
        this.rightTurretServo = turretServoRight;
        this.servoEncoder = axonServoEncoder;

        smartServoController = new RTPAxon(leftTurretServo, servoEncoder, RTPAxon.Direction.REVERSE);
        smartServoController.forceResetTotalRotation();
    }

    public void setTurretPower(double power) {
        leftTurretServo.setPower(power);
        rightTurretServo.setPower(power);
    }

    public void setTurretPowerConstrained(double power) {
        if (
                (getTurretAngle() > MAX_SERVO_ANGLE && power < 0) ||
                (getTurretAngle() < -MAX_SERVO_ANGLE && power > 0)
        ) {
            setTurretPower(0.0);
        } else {
            setTurretPower(power);
        }

    }

    public double getTurretAngle() {
        return smartServoController.getTotalRotation();
    }
}
