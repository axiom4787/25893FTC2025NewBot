package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous
public class OurAuto extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private CRServo agitator;
    float ticksPerRev = 28 * 5 * 3; //5 and 3 is the gear reductions
    float circumference = 9.5f; //in inches
    float ticksPerInch = ticksPerRev / circumference;

    public double getLowestVoltage() {
        double lowestValue = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor : hardwareMap.voltageSensor) {
            if(sensor.getVoltage() < lowestValue && sensor.getVoltage() > 0.1) {
                lowestValue = sensor.getVoltage();
            }
        }
        if(lowestValue == Double.POSITIVE_INFINITY) {
            lowestValue = 14;
        }
        telemetry.addLine("Voltage: " + lowestValue + "V");
        return lowestValue;
    }

    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        agitator = hardwareMap.get(CRServo.class, "servo");
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        feedRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            leftDrive.setTargetPosition((int)(ticksPerInch * 23));
            rightDrive.setTargetPosition((int)(ticksPerInch * 23));

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);

            while(leftDrive.isBusy() || rightDrive.isBusy()) {
                telemetry.addLine(String.valueOf(leftDrive.getCurrentPosition()));
                telemetry.addLine(String.valueOf(rightDrive.getCurrentPosition()));
                telemetry.update();
            }

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            double multiplier = 14 / getLowestVoltage();
            flywheel.setVelocity(1300 * multiplier);
            sleep(1000);
            agitator.setPower(1);
            sleep(500);
            feedRoller.setPower(1);
            sleep(6000);
            flywheel.setPower(0);
            agitator.setPower(0);
            feedRoller.setPower(0);

            break;
        }

    }
}
