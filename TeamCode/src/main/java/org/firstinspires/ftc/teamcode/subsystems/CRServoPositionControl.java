package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class CRServoPositionControl {

    //CONFIGURABLES

    // general constants
    public static double maxVoltage = 3.26;
    public static double degreesPerRev = 360.0;

    // gains (insert muscle emoji here)
    public static double kP = 0.002;
    public static double kS = 0.07;//static friction
    public static double maxPower = 1;
    public static double stiffnessGain = 1.0; // 1.0 = regular behavior
    public static double brakeZoneDeg = 20.0;
    public static double kD = 0.000;   // velocity damping
    //damping helpers
    private double lastAngleDeg = 0.0;
    private long lastTimeNs = 0;



    // deadbands
    public static double deadbandDeg = 1.5;
    //direction
    public static boolean rotateClockwise = true;

    // hardware

    private final CRServo servo;
    private final AnalogInput encoder;

    // super sigma wrapping
    private double lastWrappedDeg;
    private double continuousDeg;

    // Target
    private double targetDeg;

    //constructor

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;

        double initial = getWrappedAngle();
        lastWrappedDeg = initial;
        continuousDeg = initial;
        targetDeg = initial;

        lastAngleDeg = continuousDeg;
        lastTimeNs = System.nanoTime();

    }

    //api
    //lwkey bruno mars and rose should make APT again but make it API instead would that be tuffy
    public void update() {
        updateContinuousAngle();

        double error = targetDeg - continuousDeg;
        double absErr = Math.abs(error);

        // sigma sigma noy
        if (absErr < deadbandDeg) {
            servo.setPower(0);
            lastAngleDeg = continuousDeg;
            lastTimeNs = System.nanoTime();
            return;
        }

        // velocity measurement
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) * 1e-9;
        if (dt <= 0) dt = 1e-3;

        double velocity = (continuousDeg - lastAngleDeg) / dt; //edgrees per second

        lastAngleDeg = continuousDeg;
        lastTimeNs = now;

        double output;

        //slowing vilocty dampening near target
        if (absErr < brakeZoneDeg) {
            output = kP * error * stiffnessGain
                    - kD * velocity;
        } else {
            output = kP * error * stiffnessGain;
        }

        // apply static friction compensation ONLY when driving toward target
        if (Math.signum(output) == Math.signum(error)) {
            double sign = Math.signum(output);
            output = sign * Math.max(Math.abs(output), kS);
        }

        // clamp
        output = clamp(output, -maxPower, maxPower);

        servo.setPower(output);
    }


    //this is working good api think
    public void moveToAngle(double wrappedAngleDeg) {
        updateContinuousAngle();

        double currentWrapped = mod(continuousDeg, 360.0);

        double delta = wrappedAngleDeg - currentWrapped;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        // rotation direction
        if (rotateClockwise && delta < 0) delta += 360;
        if (!rotateClockwise && delta > 0) delta -= 360;

        targetDeg = continuousDeg + delta;
    }

    // something idk if we'll use but might have to do some refactorign if we do basically moves from current position
    public void moveBy(double deltaDeg) {
        targetDeg += deltaDeg;
    }

    public void reset() {
        double wrapped = getWrappedAngle();
        lastWrappedDeg = wrapped;
        continuousDeg = wrapped;
        targetDeg = wrapped;
        servo.setPower(0);
    }

    //bencoder (ben like a reference to ben falk who is majestic)
    private void updateContinuousAngle() {
        double wrapped = getWrappedAngle();
        double delta = wrapped - lastWrappedDeg;

        // unwrap
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        continuousDeg += delta;
        lastWrappedDeg = wrapped;
    }

    private double getWrappedAngle() {
        double v = clamp(encoder.getVoltage(), 0.0, maxVoltage);
        return (v / maxVoltage) * degreesPerRev;
    }

    /* ================= UTIL ================= */

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double mod(double v, double m) {
        double r = v % m;
        return (r < 0) ? r + m : r;
    }

    /* ================= DEBUG ================= */

    public double getCurrentAngle() {
        return continuousDeg;
    }

    public double getTargetAngle() {
        return targetDeg;
    }

    public double getTargetVoltage() {
        // wrap target angle
        double wrappedDeg = targetDeg % degreesPerRev;
        if (wrappedDeg < 0) wrappedDeg += degreesPerRev;

        // convert to voltage
        return (wrappedDeg / degreesPerRev) * maxVoltage;
    }

    public double getVoltage()
    {
        return encoder.getVoltage();
    }
}
