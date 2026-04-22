package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.Hardware;

@Configurable
public class Shooter {
    private final MotorEx shooter;

    private double targetVelocityTicks = 0;

    private enum State { OFF, IDLE, SHOOT, REVERSE };
    private State state = State.OFF;

    public Shooter() {
        shooter = Hardware.getShooterMotor();
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

    public void update(Follower follower) {
        switch (state) {
            case SHOOT:
                double distX = Globals.Misc.GOAL_X - follower.getPose().getX();
                double distY = Globals.Misc.GOAL_Y - follower.getPose().getY();
                double dist = Math.hypot(distX, distY);

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
        t.update();
    }

    /**
     * Set the target velocity to {@code vel} rounded to the nearest 20
     * @param vel - target velocity
     */
    private void setTargetVel(double vel) {
        targetVelocityTicks = Math.round(vel / 20f) * 20f;
        // Round target to nearest multiple of 20 ticks

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