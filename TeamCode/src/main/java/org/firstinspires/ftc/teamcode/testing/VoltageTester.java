package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Voltage Tester")
public class VoltageTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput signal = hardwareMap.get(AnalogInput.class, "Encoder");
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("voltage", signal.getVoltage());
            telemetry.update();
        }
    }
}
