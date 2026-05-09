package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.CommandOpMode;

public abstract class CommandOpModeWithAlliance extends CommandOpMode {
    public Alliance alliance = Alliance.RED;

    public boolean isRedAlliance() {
        return alliance == Alliance.RED;
    }

    public void selectAlliance() {
        Object action = telemetry.addAction(this::changeWarningColor);

//        telemetry.speak("twin read the DS");

        while (opModeInInit()) {
            if (gamepad1.left_bumper) alliance = Alliance.RED;
            if (gamepad1.right_bumper) alliance = Alliance.BLUE;

            telemetry.addData("Selected alliance", alliance.name());
            telemetry.addLine(getWarningMessage());
            telemetry.update();
        }

        telemetry.removeAction(action);
    }

    @Override
    public void initialize_loop() {
        selectAlliance();
    }

    private String getWarningMessage() {
        return
                "\n" +
                "<pre style='line-height:1; letter-spacing:0;'>" +
                "<font color='#" + colors[colorIndex] + "'>" +
                "  _    _            _ \n" +
                " | |  | |          | |\n" +
                " | |__| | ___ _   _| |\n" +
                " |  __  |/ _ \\ | | | |\n" +
                " | |  | |  __/ |_| |_|\n" +
                " |_|  |_|\\___|\\__, (_)\n" +
                "               __/ |  \n" +
                "              |___/   " +
                "</font>" +
                "</pre>" +
                "\n" +
                "Make sure you've chosen the correct alliance!\n\n" +
                "Left bumper: Red\n" +
                "Right bumper: Blue\n";
    }

    private static final String[] colors = {
            "ef5350", "ef5350", // red
            "448aff", "448aff", // blue
            "1dbf17", "1dbf17", // green
            "974fe0", "974fe0"  // purple
    }; // two of each because telemetry updates 4x/sec, color should change 2x per sec

    private int colorIndex = 0;

    private void changeWarningColor() {
        colorIndex = (colorIndex + 1) % colors.length;
    }
}
