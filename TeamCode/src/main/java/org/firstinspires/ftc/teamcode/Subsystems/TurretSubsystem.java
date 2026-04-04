package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.Boilerplate.RTPAxon;

public class TurretSubsystem {
    private final CRServo leftTurretServo, rightTurretServo;
    private final AnalogInput servoEncoder;
    public final RTPAxon smartServoController;
    public static final double MAX_SERVO_ANGLE = 140;
    public static final double GEAR_RATIO = -1.0; // idk what it is

    public TurretSubsystem() {
        Hardware.TurretServos turretServos = Hardware.getTurretServos();

        leftTurretServo = turretServos.left;
        rightTurretServo = turretServos.right;
        servoEncoder = Hardware.getAxonServoEncoder();

        smartServoController = new RTPAxon(leftTurretServo, servoEncoder, RTPAxon.Direction.REVERSE);
        smartServoController.forceResetTotalRotation();
        smartServoController.setRtp(true);
    }

    public void setTurretPower(double power) {
        leftTurretServo.setPower(power);
        rightTurretServo.setPower(power);
    }

    /**
     * Should be called in a loop
     */
    public void setTargetAngle(double robotHeading, double targetAngle) {
        double diff = targetAngle - (robotHeading + getTurretAngle());
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
