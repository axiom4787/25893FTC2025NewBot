package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder; // Analog input for position from 4th wire

    private double kp = 1.0;
    private double ki = 0.0;
    private double kd = 0.1;
    private double kf = 0.1;

    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        timer.reset();
    }

    private double getCurrentPositionVoltage() {
        return encoder.getVoltage(); // Read analog voltage from 4th wire
    }

    private double angleToVoltage(double angleDegrees) {
        angleDegrees = Math.max(0, Math.min(360, angleDegrees)); // Clamp
        return (angleDegrees / 360.0) * 3.3;
    }

    private double voltageToAngle(double voltage) {
        voltage = Math.max(0, Math.min(3.3, voltage));
        return (voltage / 3.3) * 360.0;
    }

    public void moveToAngle(double targetAngleDegrees) {
        double targetVoltage = angleToVoltage(targetAngleDegrees);
        double currentVoltage = getCurrentPositionVoltage();

        double error = targetVoltage - currentVoltage;

        // Shortest path wrap handling, for continuous rotation (optional)
        if (error > 1.65) { error -= 3.3; }
        if (error < -1.65) { error += 3.3; }

        double deltaTime = timer.seconds();
        timer.reset();
        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        double output = kp * error + ki * integral + kd * derivative + kf * targetVoltage;
        output = Math.max(-1.0, Math.min(1.0, output));
        crServo.setPower(output);

        lastError = error;
    }
}
