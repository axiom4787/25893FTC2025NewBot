package org.firstinspires.ftc.teamcode.testOpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.util.Context;

//@Configurable
//@TeleOp
public class TestTheBlocker extends LinearOpMode {
    public static double pos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Context.init(this);

        ServoEx blocker = Context.getBlockerServo();

        waitForStart();

        while (opModeIsActive()) {
            blocker.set(pos);
        }
    }
}
