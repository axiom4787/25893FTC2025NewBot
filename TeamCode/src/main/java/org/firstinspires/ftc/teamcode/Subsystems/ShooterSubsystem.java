package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Boilerplate.PID;
import org.firstinspires.ftc.teamcode.NewAutos.Shared2;

public class ShooterSubsystem {
    private final DcMotorEx shooter;
    private double targetVelocity = 0;
    private final PID shooterPID = new PID(800, 0, 0, 0, 1);

    private enum State { OFF, IDLE, SHOOT, REVERSE };
    private State state = State.OFF;

    public ShooterSubsystem() {
        shooter = Hardware.getShooterMotor();
    }

    public void setOff() { state = State.OFF; }
    public void setIdle() { state = State.IDLE; }
    public void setShoot() { state = State.SHOOT; }
    public void setReverse() { state = State.REVERSE; }

    public void update(Follower follower) {
        switch (state) {
            case SHOOT:
                double distX = Shared2.Misc.GOAL_X - follower.getPose().getX();
                double distY = Shared2.Misc.GOAL_Y - follower.getPose().getY();
                double dist = Math.hypot(distX, distY);

                targetVelocity = dist * 14; // very accurate math
                double vel = shooterPID.calculate(targetVelocity, getVelocity());

                shooter.setPower(vel);
                break;
            case IDLE:
                shooter.setPower(0.5);
                break;
            case REVERSE:
                shooter.setPower(-1);
                break;
            case OFF:
            default:
                shooter.setPower(0.0);
        }
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
}