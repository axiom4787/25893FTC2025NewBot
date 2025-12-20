package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
public class Shooter {
    public enum State {
        IDLE,
        SPINNING_UP,
        READY,
        FIRING
    }

    // Velocity filter
    public static double VELOCITY_ALPHA = 0.7;

    // Voltage filter
    public static double VOLTAGE_ALPHA = 0.3;

    // Feedforward constants (in VOLTS, not power)
    public static double kS = 0.3;
    public static double kV = 0.0019;

    // Feedback
    public static double kP = 0.004;

    // Target
    public static double TARGET_RPM = 2914;

    // Acceleration prediction
    public static double COAST_CONSTANT = 0.025;

    // Ready threshold
    public static double READY_THRESHOLD = 100;

    // Shot timing
    public static double TRANSFER_START_TO_CONTACT_MS = 150;
    public static double BALL_CONTACT_DURATION_MS = 80;
    public static double TRANSFER_TOTAL_DURATION_MS = 300;

    // Pre-boost buffer
    public static double BOOST_BUFFER_MS = 20;

    // Spinup timeout
    public static double SPINUP_TIMEOUT_MS = 2000;

    // Safety constants
    private static final double MIN_SAFE_VOLTAGE = 10.0;

    private DcMotorEx shooter;
    private CRServo transferTop;
    private CRServo transferBottom;
    private VoltageSensor voltageSensor;

    private State state = State.IDLE;
    private double filteredRPM = 0;
    private double lastFilteredRPM = 0;
    private double filteredVoltage = 12.0;
    private double acceleration = 0;
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean triggerHeld = false;

    public Shooter(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooter = hardwareMap.get(DcMotorEx.class, "sr");
        transferTop = hardwareMap.get(CRServo.class, "tl");
        transferBottom = hardwareMap.get(CRServo.class, "bl");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        loopTimer.reset();
    }

    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 0.02;

        double rawRPM = Math.abs((shooter.getVelocity() / 28.0) * 60.0);
        filteredRPM = (VELOCITY_ALPHA * rawRPM) + ((1 - VELOCITY_ALPHA) * filteredRPM);

        double rawVoltage = voltageSensor.getVoltage();
        filteredVoltage = (VOLTAGE_ALPHA * rawVoltage) + ((1 - VOLTAGE_ALPHA) * filteredVoltage);

        acceleration = (filteredRPM - lastFilteredRPM) / dt;
        lastFilteredRPM = filteredRPM;

        double error = TARGET_RPM - filteredRPM;

        double desiredVolts = 0;

        switch (state) {
            case IDLE:
                desiredVolts = 0;
                setTransfer(0);
                break;

            case SPINNING_UP:
                desiredVolts = filteredVoltage;
                setTransfer(0);

                if (spinUpTimer.milliseconds() > SPINUP_TIMEOUT_MS) {
                    state = State.IDLE;
                    break;
                }

                double predictedCoast = acceleration * COAST_CONSTANT;
                double predictedFinal = filteredRPM + predictedCoast;

                if (predictedFinal >= TARGET_RPM) {
                    state = State.READY;
                }
                break;

            case READY:
                desiredVolts = kS + (kV * TARGET_RPM) + (kP * error);
                setTransfer(0);

                if (Math.abs(error) > READY_THRESHOLD * 3) {
                    state = State.SPINNING_UP;
                    spinUpTimer.reset();
                }
                break;

            case FIRING:
                double t = shotTimer.milliseconds();

                double contactStart = TRANSFER_START_TO_CONTACT_MS;
                double contactEnd = contactStart + BALL_CONTACT_DURATION_MS;
                double boostStart = contactStart - BOOST_BUFFER_MS;

                if (t < TRANSFER_TOTAL_DURATION_MS) {
                    setTransfer(1);
                } else {
                    setTransfer(0);
                }

                if (t < boostStart) {
                    desiredVolts = kS + (kV * TARGET_RPM) + (kP * error);

                } else if (t < contactEnd) {
                    desiredVolts = filteredVoltage;

                } else {
                    double predictedCoastRecovery = acceleration * COAST_CONSTANT;
                    double predictedFinalRecovery = filteredRPM + predictedCoastRecovery;

                    if (predictedFinalRecovery >= TARGET_RPM) {
                        desiredVolts = kS + (kV * TARGET_RPM) + (kP * error);

                        if (error < READY_THRESHOLD) {
                            if (triggerHeld) {
                                shotTimer.reset();
                            } else {
                                state = State.READY;
                            }
                        }
                    } else {
                        desiredVolts = filteredVoltage;
                    }
                }
                break;
        }

        double safeVoltage = Math.max(filteredVoltage, MIN_SAFE_VOLTAGE);
        double power = safeClamp(desiredVolts / safeVoltage, 0, 1);
        shooter.setPower(power);
    }

    private double safeClamp(double value, double min, double max) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return 0.0;
        }
        return Math.max(min, Math.min(value, max));
    }

    public void spinUp() {
        if (state == State.IDLE) {
            state = State.SPINNING_UP;
            spinUpTimer.reset();
        }
        triggerHeld = true;
    }

    public void abort() {
        state = State.IDLE;
        triggerHeld = false;
        setTransfer(0);
    }

    public void setTriggerHeld(boolean held) {
        triggerHeld = held;
        if (!held && state != State.FIRING) {
            state = State.IDLE;
        }
    }

    public void fire() {
        if (state == State.READY) {
            state = State.FIRING;
            shotTimer.reset();
        }
    }

    private void setTransfer(double speed) {
        if (speed > 0) {
            transferTop.setPower(-1.0);
            transferBottom.setPower(1.0);
        } else if (speed < 0) {
            transferTop.setPower(1.0);
            transferBottom.setPower(-1.0);
        } else {
            transferTop.setPower(0);
            transferBottom.setPower(0);
        }
    }

    public State getState() {
        return state;
    }

    public double getRPM() {
        return filteredRPM;
    }

    public double getRawRPM() {
        return Math.abs((shooter.getVelocity() / 28.0) * 60.0);
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getVoltage() {
        return filteredVoltage;
    }

    public double getError() {
        return TARGET_RPM - filteredRPM;
    }

    public boolean isReady() {
        return state == State.READY;
    }
}
