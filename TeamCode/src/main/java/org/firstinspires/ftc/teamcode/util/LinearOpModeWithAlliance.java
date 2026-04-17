package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class LinearOpModeWithAlliance extends LinearOpMode {
    public Alliance alliance = Alliance.RED;

    public boolean isRedAlliance() {
        return alliance == Alliance.RED;
    }

    public void selectAlliance() {
        while (opModeInInit()) {
            if (gamepad1.left_bumper) alliance = Alliance.RED;
            if (gamepad1.right_bumper) alliance = Alliance.BLUE;

            telemetry.addData("Alliance", alliance.name());
            telemetry.update();
        }
    }
}
