package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.teamcode.util.Globals;

@Configurable
public class Shooter {
    private final MotorEx shooter;
    private final VoltageSensor voltageSensor;

    private static final double VOLTAGE_WHEN_TUNED = 13.5;

    private double targetVelocityTicks = 0;

    private enum State { OFF, IDLE, SHOOT, REVERSE };
    private State state = State.OFF;

    private ElapsedTime timer = new ElapsedTime();

    public Shooter() {
        shooter = Context.getShooterMotor();
        shooter.setFeedforwardCoefficients(0, 0);
        shooter.setVeloCoefficients(0, 0, 0);

        voltageSensor = Context.getVoltageSensor();
    }

    public void off() {
        state = State.OFF;
    }

    public void idle() {
        state = State.IDLE;
    }

    public void shoot() {
        state = State.SHOOT;
    }

    public void reverse() {
        state = State.REVERSE;
    }

    public void update() {
        switch (state) {
            case SHOOT:
//                if (timer.milliseconds() < 10) break;
//                timer.reset();

                double dist = Globals.distToGoal();

                if (dist < 25) {
                    // Super close to goal
                    setTargetVel(1100);
                } else if (dist > 100) {
                    // Far launch zone
                    setTargetVel(1900);
                } else {
                    // Linear interpolation from distance to velocity
                    // 25 -> 1100, 100 -> 1600
                    setTargetVel(Range.scale(dist, 25, 100, 1100, 1600));
                }
                break;
            case IDLE:
                shooter.set(0.5);
                break;
            case REVERSE:
                shooter.set(-1);
                break;
            case OFF:
            default:
                shooter.set(0.0);
        }

        log();
    }

    private void log() {
        TelemetryManager t = PanelsTelemetry.INSTANCE.getTelemetry();
        t.addData("Shooter velocity", getVelocity());
        t.addData("Shooter target", getTargetVelocity());
        t.addData("power", shooter.getRawPower());
        t.addData("voltage", voltageSensor.getVoltage());
        t.update();
    }

    /**
     * Set the target velocity to {@code vel} rounded to the nearest 20
     * @param vel target velocity in ticks
     */
    private void setTargetVel(double vel) {
        targetVelocityTicks = Math.round(vel / 20f) * 20f;
        // Round target to nearest multiple of 20 ticks

        // Todo: FF + PID with voltage compensation

        if (getError() == 0) {
            shooter.set(targetVelocityTicks / 2200f);
        } else if (getError() > 0) {
            shooter.set(1);
        } else {
            shooter.set(0);
        }
    }

    public double getError() {
        return getTargetVelocity() - getVelocity();
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocityTicks;
    }
}