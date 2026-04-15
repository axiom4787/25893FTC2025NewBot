package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Boilerplate.Shared;
import org.firstinspires.ftc.teamcode.Hardware.CachingHardware;

@Configurable
public class ShooterSubsystem {
    private final MotorEx shooter;
    private final VoltageSensor sensor;

    private double targetVelocityTicks = 0;
    public static PIDFController pidf = new PIDFController(0.04, 0, 0.00001, 0.00002);

    private enum State { OFF, IDLE, SHOOT, REVERSE };
    private State state = State.IDLE;

    public ShooterSubsystem() {
        shooter = CachingHardware.getShooterMotor();
        sensor = CachingHardware.getVoltageSensor();
    }

    public void setOff() {
        state = State.OFF;
    }

    public void setIdle() {
        state = State.IDLE;
    }

    public void setShoot() {
        state = State.SHOOT;
    }

    public void setReverse() {
        state = State.REVERSE;
    }

    public void update(Follower follower) {
        switch (state) {
            case SHOOT:
                double distX = Shared.Misc.GOAL_X - follower.getPose().getX();
                double distY = Shared.Misc.GOAL_Y - follower.getPose().getY();
                double dist = Math.hypot(distX, distY);

                if (dist < 25) {
                    setTargetVel(1300);
                } else if (dist > 100) {
                    setTargetVel(2000);
                } else {
                    setTargetVel(Range.scale(dist, 25, 100, 1300, 1700));
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

    public void update(Follower follower, Gamepad gp) {
        double[] coeffs = shooter.getFeedforwardCoefficients();
        double kV = 0; // as tuned at ??? volts
        double voltage = sensor.getVoltage();

//        shooter.setFeedforwardCoefficients(coeffs[0], kV/(voltage/13.15), coeffs[2]);

        if (gp.y) {
            targetVelocityTicks = 1000;
        } else if (gp.b) {
            targetVelocityTicks = 1500;
        } else if (gp.a) {
            targetVelocityTicks = 1900;
        } else if (gp.x) {
            targetVelocityTicks = 0;
        } else if (gp.dpad_up) {
            targetVelocityTicks = 2230;
        }

//        double vel = shooter.getVelocity();
//        if (targetVelocityTicks - vel > 50) {
//            shooter.set(1);
//        } else {
//            shooter.setVelocity(targetVelocityTicks);
//        }
        shooter.setVelocity(targetVelocityTicks);

        log();
    }

    private void log() {
        TelemetryManager t = PanelsTelemetry.INSTANCE.getTelemetry();
        t.addData("Shooter velocity", getVelocity());
        t.addData("Shooter target", getTargetVelocity());
        t.addData("power", shooter.getRawPower());
        t.addData("v", sensor.getVoltage());
        t.update();
    }

    private void setTargetVel(double vel) {
        boolean no = false;

        targetVelocityTicks = vel;
        // lowkey seems to work fine
        if (getVelocity() < targetVelocityTicks) {
            shooter.set(1);
        } else {
            shooter.set(0);
        }
//        if (targetVelocityTicks - getVelocity() > 60 && no) {
////            shooter.setRunMode(Motor.RunMode.RawPower);
//            shooter.set(1);
//        } else if (targetVelocityTicks - getVelocity() < -60 && no) {
////            shooter.setRunMode(Motor.RunMode.RawPower);
//            shooter.set(0);
//        } else {
////            shooter.setRunMode(Motor.RunMode.RawPower);
//            double power = pidf.calculate(getVelocity(), targetVelocityTicks);
//            shooter.set(power < 0 ? 0 : power);
//        }
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocityTicks;
    }
}