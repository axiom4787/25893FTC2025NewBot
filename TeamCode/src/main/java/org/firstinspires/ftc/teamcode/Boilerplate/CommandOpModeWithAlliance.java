package org.firstinspires.ftc.teamcode.Boilerplate;

import com.seattlesolvers.solverslib.command.CommandOpMode;

public abstract class CommandOpModeWithAlliance extends CommandOpMode {
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

    @Override
    public void initialize_loop() {
        selectAlliance();
    }
}
