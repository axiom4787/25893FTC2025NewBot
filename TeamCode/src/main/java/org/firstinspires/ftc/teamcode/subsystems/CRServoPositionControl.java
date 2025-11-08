package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
// TODO: this is vibe coded so probably needs testing and adjustments
public class CRServoPositionControl {
    private CRServo crServo;
    private double kp = 1.0; // Proportional gain
    private double ki = 0.0; // Integral gain
    private double kd = 0.1; // Derivative gain
    private double kf = 0.1; // Feedforward gain

    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    public CRServoPositionControl(CRServo servo) {
        this.crServo = servo;
        timer.reset();
    }

    // Read encoder voltage from hardware (0 to 3.3V)


    // Convert angle (0 to 360 degrees) to voltage (0 to 3.3V)
    private double angleToVoltage(double angleDegrees) {
        angleDegrees = Math.max(0, Math.min(360, angleDegrees)); // Clamp angle
        return (angleDegrees / 360.0) * 3.3;
    }

    // Move to target angle in degrees using PIDF control
    public void moveToAngle(double targetAngleDegrees) {
        double targetVoltage = angleToVoltage(targetAngleDegrees);
        double currentVoltage = crServo.getPower();

        double error = targetVoltage - currentVoltage;

        double deltaTime = timer.seconds();
        timer.reset();

        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        // PIDF output for power command [-1, 1]
        double output = kp * error + ki * integral + kd * derivative + kf * targetVoltage;
        output = Math.max(-1.0, Math.min(1.0, output));

        crServo.setPower(output);

        lastError = error;
    }


}
