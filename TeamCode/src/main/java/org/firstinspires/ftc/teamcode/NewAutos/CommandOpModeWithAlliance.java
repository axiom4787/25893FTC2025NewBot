package org.firstinspires.ftc.teamcode.NewAutos;

import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Boilerplate.Alliance;

public abstract class CommandOpModeWithAlliance extends CommandOpMode {
    public Alliance alliance = Alliance.RED;

    public boolean isRedAlliance() {
        return alliance == Alliance.RED;
    }

    public void selectAlliance() {
        while (opModeInInit()) {
            if (gamepad1.right_bumper) alliance = Alliance.RED;
            if (gamepad1.left_bumper) alliance = Alliance.BLUE;

            telemetry.addData("Alliance", alliance.name());
            telemetry.update();
        }
    }

    @Override
    public void initialize_loop() {
        selectAlliance();
    }
}
