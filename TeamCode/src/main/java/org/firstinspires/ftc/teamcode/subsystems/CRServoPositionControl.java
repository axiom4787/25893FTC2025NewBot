package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl
{
    private final CRServo crServo;
    private final AnalogInput encoder;
    private ElapsedTime timer = new ElapsedTime();

    // Gains
    public static double kp = 0.41;
    public static double ki = 0.0;
    public static double kf = 0.01;
    public static double filterAlpha = 0.9;

    // Encoder constants
    public static double ticksPerRev = 3.3;     // full voltage range
    public static double degreesPerRev = 360.0;

    // GLOBAL OFFSET APPLIED ONLY DURING CONTROL
    public static double constantOffset = 0.15;

    // Internal state
    private double integral = 0;
    private double filteredVoltage = 0;

    private double targetVoltage_actual = 0;   // true target
    private double targetVoltage_offset = 0;   // target + offset

    public CRServoPositionControl(CRServo servo, AnalogInput encoder)
    {
        this.crServo = servo;
        this.encoder = encoder;
    }

    public void moveToAngle(double targetAngleDegrees)
    {
        // Real target voltage
        targetVoltage_actual = angleToVoltage(targetAngleDegrees);

        // Offset target voltage used by PID
        targetVoltage_offset = applyOffset(targetVoltage_actual);

        // Measured voltage (raw)
        double measured = getFilteredVoltage();

        // Offset measured voltage used by PID
        double measured_offset = applyOffset(measured);

        // Compute shortest-path error
        double error = shortestError(targetVoltage_offset, measured_offset);

        double dt = timer.seconds();
        timer.reset();
        if (dt < 1e-4) dt = 1e-4;

        integral += error * dt;
        integral = Math.max(-2, Math.min(2, integral));

        double output = kp * error + ki * integral + kf * Math.signum(error);
        output = Math.max(-1, Math.min(1, output));

        crServo.setPower(output);
    }
    private double lastRawVoltage = 0;
    private double unwrappedVoltage = 0;

    private double getFilteredVoltage()
    {
        double raw = encoder.getVoltage();

        double diff = raw - lastRawVoltage;

        // unwrap
        if (diff > ticksPerRev/2) diff -= ticksPerRev;
        if (diff < -ticksPerRev/2) diff += ticksPerRev;

        unwrappedVoltage += diff;
        lastRawVoltage = raw;

        // NOW filter the unwrapped value
        filteredVoltage =
                (1 - filterAlpha) * filteredVoltage +
                        filterAlpha * unwrappedVoltage;

        // re-wrap filtered value
        double wrapped = filteredVoltage % ticksPerRev;
        if (wrapped < 0) wrapped += ticksPerRev;

        return wrapped;
    }
    private double applyOffset(double voltage)
    {
        double v = voltage + constantOffset;
        if (v >= ticksPerRev) v -= ticksPerRev;
        if (v < 0) v += ticksPerRev;
        return v;
    }

    private double shortestError(double target, double current)
    {
        double diff = target - current;

        if (diff > ticksPerRev / 2) diff -= ticksPerRev;
        if (diff < -ticksPerRev / 2) diff += ticksPerRev;

        return diff;
    }

    private double angleToVoltage(double angleDegrees)
    {
        double a = Math.max(0, Math.min(degreesPerRev, angleDegrees));
        return (a / degreesPerRev) * ticksPerRev;
    }
    public double getActualTargetVoltage()
    {
        return targetVoltage_actual;
    }

    public double getOffsetTargetVoltage()
    {
        return targetVoltage_offset;
    }

    public double getOffsetMeasuredVoltage()
    {
        return applyOffset(getFilteredVoltage());
    }
    public double getCurrentAngle()
    {
        double realVoltage = getFilteredVoltage();
        return (realVoltage / ticksPerRev) * degreesPerRev;
    }

}

/*
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder;

    private static boolean angleIsLocked = false;
    private static double lockedAngle;
    public static double kp = 0.341;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0167;
    public static double lockedkp = 0.341;
    public static double lockedki = 0.0;
    public static double lockedkd = 0.0;
    public static double lockedkf = 0.0167;
    public static double filterAlpha = 0.80;
    public static double angleDeadband = 1.67;


    // Dynamic speed parameters
    public static double minSpeed = 0.15;           // minimal power to overcome deadband
    public static double maxErrorForScaling = 90.0; // error threshold for full speed

    private double integral = 0.0;
    private double lastError = 0.0;
    private Double filteredVoltage = null;
    public final double MAX_VOLTAGE;
    private ElapsedTime timer = new ElapsedTime();
    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        this.MAX_VOLTAGE = encoder.getMaxVoltage();
        timer.reset();
    }

    private double getFilteredVoltage() {
        if (filteredVoltage == null) {
            filteredVoltage = encoder.getVoltage();  // initialize to first reading
        } else {
            filteredVoltage = (1 - filterAlpha) * filteredVoltage + filterAlpha * encoder.getVoltage();
        }
        return filteredVoltage;
    }

    double getAngle() {
        return (getFilteredVoltage() / MAX_VOLTAGE) * 360.0;
    }

    public void lockAngle(double targetAngleDegrees) {
        angleIsLocked = true;
        lockedAngle = targetAngleDegrees;
    }
    public void unlockAngle() {
        angleIsLocked = false;
    }

    public void moveToAngle(double targetAngleDegrees) {
        double tempkp = kp;
        double tempki = ki;
        double tempkd = kd;
        double tempkf = kf;
        if (angleIsLocked) {
            targetAngleDegrees = lockedAngle;
            tempkp = lockedkp;
            tempki = lockedki;
            tempkd = lockedkd;
            tempkf = lockedkf;
        }

        double currentAngle = getAngle();
        double error = ((targetAngleDegrees - currentAngle + 540) % 360) - 180;

        if (Math.abs(error) < angleDeadband) {
            crServo.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime <= 0.0001) deltaTime = 0.0001;

        integral += error * deltaTime;
        integral = Math.max(-2, Math.min(2, integral));

        double derivative = (error - lastError) / deltaTime;

        double output = tempkp * error + tempki * integral + tempkd * derivative + tempkf * Math.signum(error);

        // Dynamic speed scaling, speeds up if distance is further
        double distanceFactor = Math.min(Math.abs(error) / maxErrorForScaling, 1.0);
        double scaledPower = minSpeed + (1.0 - minSpeed) * distanceFactor;
        output = Math.signum(output) * Math.min(Math.abs(output), scaledPower);

        crServo.setPower(output);
        lastError = error;
    }
}
*/