package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test: Limelight vs. HuskyLens", group="Experiments")
public class TestLimelightVsHuskyLens extends LinearOpMode {
    private Limelight3A limelight;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(1);
        limelight.pipelineSwitch(0);
        limelight.start();

        huskyLens = hardwareMap.get(HuskyLens.class, "ebk");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        int ll_seen = 0;
        int hl_seen = 0;
        while (opModeIsActive()) {
            telemetry.addData(">", "Robot Running.");
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                ll_seen += 1;
            }

            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0) {
                hl_seen += 1;
            }

            telemetry.addLine();
            telemetry.addData("LL Seen", ll_seen);
            telemetry.addData("HL Seen", hl_seen);
            telemetry.update();
        }
    }
}