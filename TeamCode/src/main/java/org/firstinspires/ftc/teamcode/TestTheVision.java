package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptSoundsASJava;

@TeleOp(name="Test what vision do visiony things (TM)", group="Linear OpMode")
public class TestTheVision extends LinearOpMode {
    private CRServo turretLeft, turretRight;
    private HuskyLens huskyLens;
    Config config = new Config();
    double amt = 0;

    @Override
    public void runOpMode() {
        initElectronics();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        setTurretServosPosition(0.5);

        // run until the end of the match (driver presses STOP)
        double P = 0;
        double D = 0;

        while (opModeIsActive()) {
            turret();

            // Show the elapsed game time and wheel power.
            telemetry.update();
        }
    }
    private void initElectronics() {
        config.init(hardwareMap);

        turretLeft = config.turretServoLeft;
        turretRight = config.turretServoRight;
        huskyLens = config.huskyLens;
    }
    HuskyLens.Block target;
    private void turret() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        double size = 0f;
        for (int i = 0; i < blocks.length; i++) {
            HuskyLens.Block block = blocks[i];
            if (block.height * block.width > size) {
                target = block;
                size = block.height * block.width;
            }
            telemetry.addLine();
            telemetry.addData("Block", block.toString());
        }

        if (target != null) {
            setTurretServosPosition(-((target.x / 160f) - 1) * 0.5);
        } else {
            setTurretServosPosition(0f);
        }

    }
    private void setTurretServosPosition(double position) {
        turretRight.setPower(position);
        turretLeft.setPower(position);
    }
}
