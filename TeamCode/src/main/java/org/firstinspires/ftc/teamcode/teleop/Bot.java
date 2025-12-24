package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
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

    private final GamepadEx g1;
    private final GamepadEx g2;
    private final Telemetry telemetry;

    private boolean fieldCentric = false;

    public enum FSM {
        Intake,
        QuickOuttake,
        SortOuttake,
        Endgame
    }

    public FSM state;

    private static final double TRIGGER_DEADZONE = 0.05;
    private static final double SHOOTER_RPM = 5000;
    private static final double NON_INDEX_SPIN_TIME = 0.6;//seconds of full-power indexer blast
    private static final double SHOOTER_SPINUP = 0.75;

    public Bot(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        movement = new Movement(hardwareMap);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        telemetry = tele;
        state = FSM.Intake;
    }

    public void teleopInit() {
        indexer.initializeColors(Indexer.ArtifactColor.EMPTY);
        indexer.moveTo(Indexer.IndexerState.zero);
        indexer.setIntaking(true);
        state = FSM.Intake;
    }

    public void teleopTick() {
        g1.readButtons();
        g2.readButtons();

        handleMovement();

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            fieldCentric = !fieldCentric;
        }

        switch (state) {
            case Intake -> handleIntakeState();
            case QuickOuttake -> handleQuickOuttakeState();
            case SortOuttake -> handleSortOuttakeState();
            case Endgame -> handleEndgameState();
        }

        outtake.periodic();
        indexer.update();

        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Indexer State", "%s -> %s", indexer.getState(), indexer.getState().next());
        telemetry.addData("Indexer Voltages", "Target: %.3f , Actual: %.3f", indexer.getTargetVoltage(), indexer.getVoltage());
        telemetry.addData("Outtake RPM", "Target: %.1f, Actual: %.1f", outtake.getTargetRPM(), outtake.getRPM());
        telemetry.addData("Actuator up?", actuator.isActivated());
        telemetry.addData("Indexer Loaded?", indexer.isLoaded());
        telemetry.update();
    }

    private void handleMovement() {
        double lx = g1.getLeftX();
        double ly = g1.getLeftY();
        double rx = g1.getRightX();

        if (fieldCentric) movement.teleopTickFieldCentric(lx, ly, rx, 0, true);
        else movement.teleopTick(lx, ly, rx, 0);
    }

    private void handleIntakeState() {
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        if (leftTrigger > TRIGGER_DEADZONE) intake.run();
        else intake.stop();

        if (g2.wasJustPressed(GamepadKeys.Button.A)) state = FSM.QuickOuttake;
        if (g2.wasJustPressed(GamepadKeys.Button.B)) state = FSM.SortOuttake;
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) state = FSM.Endgame;
    }

    private void handleQuickOuttakeState() {
        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            Actions.runBlocking(actionNonIndexedDump());
        }
        if (g2.wasJustPressed(GamepadKeys.Button.A)) state = FSM.Intake;
    }

    private void handleSortOuttakeState() {
        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            Actions.runBlocking(fireWithPeriodic(actionFireGreen()));
        }
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) {
            Actions.runBlocking(fireWithPeriodic(actionFirePurple()));
        }
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            state = FSM.Intake;
        }
    }

    private void handleEndgameState() {
        if (g2.wasJustPressed(GamepadKeys.Button.A)) state = FSM.Intake;
    }

    public Action periodics() {
        return new ParallelAction(
                new InstantAction(outtake::periodic),
                new InstantAction(indexer::update)
        );
    }

    private Action actionNonIndexedDump() {
        return new SequentialAction(
                new InstantAction(() -> indexer.setIntaking(false)),
                new InstantAction(actuator::upQuick),                 // lower up position for quick dump
                new InstantAction(() -> outtake.set(SHOOTER_RPM)),
                new SleepAction(SHOOTER_SPINUP),                      // spin up shooter
                new InstantAction(() -> indexer.setIndexerPower(1.0)),// full blast
                new SleepAction(NON_INDEX_SPIN_TIME),
                new InstantAction(indexer::stopIndexerPower),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.zero)),
                new InstantAction(() -> indexer.setIntaking(true))
        );
    }

    private Action actionFireGreen() {
        return new SequentialAction(
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveToColor(Indexer.ArtifactColor.GREEN)),
                new InstantAction(() -> outtake.set(SHOOTER_RPM)),
                new SleepAction(2.0),
                new InstantAction(actuator::upIndexed),// higher position for indexed firing
                new SleepAction(0.5),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down)
        );
    }

    private Action actionFirePurple() {
        return new SequentialAction(
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveToColor(Indexer.ArtifactColor.PURPLE)),
                new InstantAction(() -> outtake.set(SHOOTER_RPM)),
                new SleepAction(2.0),
                new InstantAction(actuator::upIndexed),
                new SleepAction(0.75),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down)
        );
    }

    private Action fireWithPeriodic(Action fireAction) {
        return packet -> {
            indexer.update();
            outtake.periodic();
            return fireAction.run(packet);
        };
    }
}