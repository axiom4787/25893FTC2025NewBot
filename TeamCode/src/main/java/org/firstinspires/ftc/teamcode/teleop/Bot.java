package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class Bot {
    private final Intake intake;
    private final Indexer indexer;
    private final Actuator actuator;
    private final Outtake outtake;
    private final Movement movement;

    private final AprilTag aprilTag;
    private final AprilTagAimer aprilAimer;
    private final GamepadEx g1;
    private final GamepadEx g2;
    private final Telemetry telemetry;

    private long lastAimUpdate = 0;
    private double lastTurnCorrection = 0;

    private boolean continuousAprilTagLock = false;
    private boolean fieldCentric = false;

    private static final long AIM_UPDATE_INTERVAL_MS = 50;

    public Bot(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap);
        movement = new Movement(hardwareMap);

        aprilTag = new AprilTag(hardwareMap);
        aprilAimer = new AprilTagAimer(hardwareMap);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        telemetry = tele;
    }
    public void teleopInit() {
        indexer.startIntake();
    }
    public void teleopTick() {
        g1.readButtons();
        g2.readButtons();

        //apriltag turn correction
        double turnCorrection = 0;

        if (continuousAprilTagLock) {
            long now = System.currentTimeMillis();

            if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
                lastAimUpdate = now;

                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();

                if (!Double.isNaN(bearing)) {
                    lastTurnCorrection = aprilAimer.calculateTurnPowerToBearing(bearing);
                } else {
                    lastTurnCorrection = 0;
                }
            }

            turnCorrection = 0.9 * lastTurnCorrection;  // smooth decay
        }

        //drivetrain control
        if (fieldCentric) {
            movement.teleopTickFieldCentric(
                    g1.getLeftX(),
                    g1.getLeftY(),
                    g1.getRightX(),
                    turnCorrection,
                    true
            );
        } else {
            movement.teleopTick(
                    g1.getLeftX(),
                    g1.getLeftY(),
                    g1.getRightX(),
                    turnCorrection
            );
        }

        // Toggle field centric
        if (g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) fieldCentric = true;
        if (g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) fieldCentric = false;



        // intake control
        if (indexer.getIntaking()) {
            if(g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.01){
                intake.run();
            }
            else {
                intake.stop();
            }
        }

        //outtake control
        if (!indexer.getIntaking()) {
            if (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.01) {
                outtake.run();
            } else {
                outtake.stop();
            }
        }

        // spindexer control
        // Advance state
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            if (!indexer.isBusy()) indexer.moveTo(indexer.nextState());
        }

        //actuator control
        if (!indexer.getIntaking()) {
            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                actuator.up();
            }
            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                actuator.down();
            }
        }

        // Scan obelisk
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            aprilTag.scanObeliskTag();
            telemetry.addData("Obelisk ID", aprilTag.getObeliskId());
        }

        // Set intaking ON
        if (g2.wasJustPressed(GamepadKeys.Button.A) && !actuator.isActivated()) {
            if (!indexer.isBusy()) indexer.setIntaking(true);
        }

        // Set intaking OFF
        if (g2.wasJustPressed(GamepadKeys.Button.B)) {
            if (!indexer.isBusy()) indexer.setIntaking(false);
        }

        indexer.update();

        // Toggle continuous lock
        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            continuousAprilTagLock = !continuousAprilTagLock;
            if (continuousAprilTagLock) aprilTag.setCurrentCameraScannedId(0);
        }

        // Alliance selection
        if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            aprilTag.setGoalTagID(20); // blue
        if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            aprilTag.setGoalTagID(24); // red

        // ========== TELEMETRY ==========
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("April Lock", continuousAprilTagLock);
        telemetry.addData("Turn Correction", turnCorrection);
        telemetry.addData("Indexer State", indexer.getState());
        telemetry.addData("Next State", indexer.nextState());
        telemetry.addData("Indexer Voltage", indexer.getVoltageAnalog());
        telemetry.addData("Outtake Power", outtake.getPower());
        telemetry.update();
    }
}
