package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.FlyWheelTesterOpMode;

public class YeeterKing {

    private boolean isYeeting;
    private DcMotorEx yeetWheelLeft;
    private DcMotorEx yeetWheelNotLeft;

    final double tolerance = 0.1;

    private enum LaunchState {
        /**
         * The default state
         */
        IDLE,

        SPIN_UP,

        LAUNCH,
        /**
         * In this state, we are actively waiting for the launch to occur.
         * From here, we probably need to return to SPIN_UP to get back to launch
         * velocity.
         */
        LAUNCHING
    }

    private LaunchState launchState = LaunchState.IDLE;

    public void init(HardwareMap hardwareMap) {
        yeetWheelLeft = hardwareMap.get(DcMotorEx.class, "flywheel_left");

        yeetWheelLeft.setDirection(DcMotor.Direction.FORWARD);

        yeetWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        yeetWheelNotLeft = hardwareMap.get(DcMotorEx.class, "flywheel_right");

        yeetWheelNotLeft.setDirection(DcMotor.Direction.REVERSE);

        yeetWheelNotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients coefficients = new PIDFCoefficients(300, 0, 0, 10);
        yeetWheelLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        yeetWheelNotLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

    }

    public void launch(boolean shotRequested, double shootVelocity) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }

                break;
            case SPIN_UP:
                yeetWheelLeft.setVelocity(shootVelocity);
                yeetWheelNotLeft.setVelocity(shootVelocity);

                // Advance to LAUNCH when both motors are up to speed
                if (yeetWheelLeft.getVelocity() >= (shootVelocity - tolerance) &&
                        yeetWheelNotLeft.getVelocity() >= (shootVelocity - tolerance)) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                yeetWheelLeft.setVelocity(shootVelocity);
                yeetWheelNotLeft.setVelocity(shootVelocity);
                // For now, if velocity drops on either flyWheel, we probably fired...
                // go back to SPIN_UP
                if (yeetWheelLeft.getVelocity() < (shootVelocity + tolerance) ||
                        yeetWheelNotLeft.getVelocity() < (shootVelocity + tolerance)) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

        }


    }
}