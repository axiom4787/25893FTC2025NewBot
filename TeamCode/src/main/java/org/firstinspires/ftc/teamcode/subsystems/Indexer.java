package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Indexer {

    public enum ArtifactColor { unknown, purple, green }

    public enum IndexerState {
        zero(0),
        one(1),
        two(2);

        public final int index;

        IndexerState(int index) {
            this.index = index;
        }

        public IndexerState next() {
            return values()[(index + 1) % values().length];
        }
    }

    //dashboard controlling nd such
    public static boolean dashAdvance = false;

    public static int dashTargetSlot = -1; // -1 = disabled and 0/1/2 = slot

    // config

    public static double offsetAngle = 17;
    public static double outtakeOffsetAngle = 5;

    // scan timing
    private static final double msPerDegree = 0.6;
    private static final double minWait = 100;
    private static final double maxWait = 300;

    // objects

    private final ColorSensorSystem colorSensor;
    private final CRServoPositionControl servoControl;

    // internal state

    private IndexerState state = IndexerState.zero;
    private boolean intaking = true;

    private final ArtifactColor[] artifacts = {
            ArtifactColor.unknown,
            ArtifactColor.unknown,
            ArtifactColor.unknown
    };

    // scan scheduling
    private final ElapsedTime scanTimer = new ElapsedTime();
    private boolean scanPending = false;
    private double scanDelayMs;

    // dashboard edge detection
    private boolean lastDashAdvance = false;
    private int lastDashTargetSlot = -1;

    public Indexer(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.get(CRServo.class, "index");
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "indexAnalog");

        servoControl = new CRServoPositionControl(servo, analog);
        colorSensor = new ColorSensorSystem(hardwareMap);
    }

    // getters

    public IndexerState getState() {
        return state;
    }

    public boolean isIntaking() {
        return intaking;
    }

    public boolean isBusy() {
        return scanPending;
    }

    public double getVoltage()
    {
        return servoControl.getVoltage();
    }

    public double getTargetVoltage()
    {
        return servoControl.getTargetVoltage();
    }

    public ArtifactColor getColorAt(IndexerState s) {
        return artifacts[s.index];
    }

    //api
    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(state);
        }
    }

    public void moveToColor(ArtifactColor color) {
        for (IndexerState s : IndexerState.values()) {
            if (artifacts[s.index] == color) {
                moveTo(s);
                return;
            }
        }
    }

    // movement
    public void moveTo(IndexerState newState) {
        if (newState == state && !scanPending) return;
        // Slot = 0, 1, 2
        //120 per position
        double wrappedTargetAngle = newState.index * 120.0;

        if (!intaking) {
            wrappedTargetAngle += outtakeOffsetAngle;
        }

        wrappedTargetAngle += offsetAngle;

        // Normalize to 0, 360
        wrappedTargetAngle = mod(wrappedTargetAngle, 360.0);

        //they see me rolling
        double currentWrapped = mod(
                servoControl.getCurrentAngle(),
                360.0
        );

        double deltaCW = wrappedTargetAngle - currentWrapped;
        if (deltaCW < 0) deltaCW += 360.0;

        scanDelayMs = clamp(
                deltaCW * msPerDegree,
                minWait,
                maxWait
        );

        scanTimer.reset();
        scanPending = true;

        servoControl.moveToAngle(wrappedTargetAngle);
        state = newState;
    }

    public void update() {
        if (dashAdvance && !lastDashAdvance) {
            moveTo(state.next());
        }
        lastDashAdvance = dashAdvance;

        if (dashTargetSlot != lastDashTargetSlot) {
            if (dashTargetSlot >= 0 && dashTargetSlot <= 2) {
                moveTo(IndexerState.values()[dashTargetSlot]);
            }
            lastDashTargetSlot = dashTargetSlot;
        }

        if (scanPending && scanTimer.milliseconds() >= scanDelayMs) {
            artifacts[state.index] = colorSensor.getColor();
            scanPending = false;
        }

        servoControl.update();
    }

    //util

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double mod(double v, double m) {
        double r = v % m;
        return r < 0 ? r + m : r;
    }
}
