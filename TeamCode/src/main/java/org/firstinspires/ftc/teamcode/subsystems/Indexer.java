package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Indexer {

    public enum ArtifactColor { unknown, purple, green }
    public enum IndexerState { one, two, three }

    // config
    public static final double DEADBAND = 1.67;     // degrees
    public static double offsetAngle = 105;         // offset degrees
    public static double outtakeOffsetAngle = 5;    // extra offset for outtake
    public static double targetAngle = 0;

    // scan timing
    private final double msPerDegree = 0.6;
    private final double minWait = 100;
    private final double maxWait = 300;

    // objects
    private final ColorSensorSystem colorSensor;
    private final CRServoPositionControl indexerServoControl;
    private final AnalogInput indexerAnalog;
    private final Actuator actuator;

    // internal state
    private IndexerState state = IndexerState.one;
    private boolean intaking = true;

    private ArtifactColor[] artifacts = {
            ArtifactColor.unknown,
            ArtifactColor.unknown,
            ArtifactColor.unknown
    };

    private final ElapsedTime scanTimer = new ElapsedTime();
    private boolean scanPending = false;
    private double scanDelay;


    // constructor
    public Indexer(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.get(CRServo.class, "index");
        indexerAnalog = hardwareMap.get(AnalogInput.class, "indexAnalog");

        actuator = new Actuator(hardwareMap);
        indexerServoControl = new CRServoPositionControl(servo, indexerAnalog);
        colorSensor = new ColorSensorSystem(hardwareMap);
    }


    // getters and state logic
    public IndexerState getState() { return state; }
    public boolean getIntaking() { return intaking; }
    public boolean isBusy() { return scanPending; }
    public double getVoltageAnalog() { return indexerAnalog.getVoltage(); }
    public double getTargetVoltage() { return indexerServoControl.getTargetVoltage(); }


    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(state); // recompute targetAngle for new mode
        }
    }

    public void startIntake() {
        intaking = true;
        moveTo(nextState());
    }

    public void startOuttake() {
        intaking = false;
        moveTo(nextState());
    }


    // scan handling
    public ArtifactColor stateToColor(IndexerState s) {
        int idx = stateToNum(s) - 1;
        return artifacts[idx];
    }

    public void scanArtifact() {
        ArtifactColor scanned = colorSensor.getColor();
        int idx = stateToNum(IndexerState.one) - 1;  // sensor position always slot 1
        artifacts[idx] = scanned;
    }

    // shifts artifact arrangements to match rotation direction
    public void shiftArtifacts(IndexerState from, IndexerState to) {
        int oldIdx = stateToNum(from) - 1;
        int newIdx = stateToNum(to) - 1;

        int diff = newIdx - oldIdx;

        if (diff < 0) diff += 3;
        if (diff > 0) diff %= 3;

        // Rotate array diff times (each rotation is one "slot" shift)
        for (int i = 0; i < diff; i++) {
            artifacts = new ArtifactColor[]{
                    artifacts[2], artifacts[0], artifacts[1]
            };
        }

        // Clear slot if actuator fired
        if (actuator.isActivated()) {
            artifacts[newIdx] = ArtifactColor.unknown;
        }
    }


    // movement
    public void moveToColor(ArtifactColor color) {
        if (stateToColor(IndexerState.one) == color) moveTo(IndexerState.one);
        else if (stateToColor(IndexerState.two) == color) moveTo(IndexerState.two);
        else if (stateToColor(IndexerState.three) == color) moveTo(IndexerState.three);
    }

    /** Moves the indexer to a new state (sets targetAngle only). */
    public void moveTo(IndexerState newState) {

        shiftArtifacts(state, newState);

        double actualAngle = indexerServoControl.getCurrentAngle();
        int slot = stateToNum(newState) - 1;

        targetAngle = slot * 120;

        if (!intaking) targetAngle += outtakeOffsetAngle;

        targetAngle = (targetAngle + offsetAngle) % 360;

        double delta = Math.abs(targetAngle - actualAngle);
        if (delta > 180) delta = 360 - delta;
        if (delta < DEADBAND) return;

        // Schedule scanning
        double wait = Math.min(maxWait, Math.max(minWait, delta * msPerDegree));
        scanTimer.reset();
        scanDelay = wait;
        scanPending = true;

        state = newState;
    }

    /** Call every OpMode loop */
    public void update() {
        indexerServoControl.moveToAngle(targetAngle);

        if (scanPending && scanTimer.milliseconds() >= scanDelay) {
            scanArtifact();
            scanPending = false;
        }
    }

    // utils

    public IndexerState nextState() {
        int num = stateToNum(state);
        return numToState((num % 3) + 1);
    }

    public IndexerState numToState(int num) {
        switch (num) {
            case 1: return IndexerState.one;
            case 2: return IndexerState.two;
            case 3: return IndexerState.three;
        }
        return IndexerState.one;
    }

    public int stateToNum(IndexerState s) {
        switch (s) {
            case one:   return 1;
            case two:   return 2;
            case three: return 3;
        }
        return 1;
    }
}
