package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Indexer {

    //enums (theres too many)

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        EMPTY,
        UNKNOWN
    }

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

    public static double offsetAngle = 17.0;
    public static double outtakeOffsetAngle = 180.0;

    //slot spacing for color sensing
    private static final double SLOT_SPACING_DEG = 120.0;
    private static final double SLOT_ASSIGN_TOLERANCE = 15.0;


    // scan timing
    private static final double msPerDegree = 0.6;
    private static final double minWait = 100;
    private static final double maxWait = 300;

    // Separate percentage thresholds for each color state (THESE WILL BE LOWERED SIGNFICANTLY)
    public static double GREEN_THRESHOLD = 0.6;
    public static double PURPLE_THRESHOLD = 0.6;
    public static double EMPTY_THRESHOLD = 0.6;
    public static double UNKNOWN_THRESHOLD = 0.6;


    // Telemetry object
    private Telemetry telemetry = null;
    public void setTelemetry(Telemetry t) { telemetry = t; }


    // objects

    private final ColorSensorSystem colorSensor;
    private final CRServoPositionControl servoControl;

    // internal state

    private IndexerState state = IndexerState.zero;
    private boolean intaking = true;

    private final ArtifactColor[] artifacts = {
            ArtifactColor.UNKNOWN,
            ArtifactColor.UNKNOWN,
            ArtifactColor.UNKNOWN
    };

    private final SlotObservation[] observations = {
            new SlotObservation(),
            new SlotObservation(),
            new SlotObservation()
    };
    private IndexerState lastClosestSlot = null;


    // scan scheduling
    private final ElapsedTime scanTimer = new ElapsedTime();
    private boolean scanPending = false;
    private double scanDelayMs;

    // dashboard edge detection
    private boolean lastDashAdvance = false;
    private int lastDashTargetSlot = -1;

    //const
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

    public ArtifactColor getColorAt(IndexerState s) {
        return artifacts[s.index];
    }

    public double getMeasuredAngle() {
        return mod(servoControl.getCurrentAngle(), 360.0);
    }

    //api

    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(state);
        }
    }

    public void moveTo(IndexerState newState) {
        if (newState == state && !scanPending) return;

        double targetAngle = getSlotCenterAngle(newState);

        double currentWrapped = getMeasuredAngle();
        double deltaCW = targetAngle - currentWrapped;
        if (deltaCW < 0) deltaCW += 360.0;

        scanDelayMs = clamp(deltaCW * msPerDegree, minWait, maxWait);

        scanTimer.reset();
        scanPending = true;

        servoControl.moveToAngle(targetAngle);
        state = newState;
    }
    //update loop
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

        servoControl.update();

        IndexerState currentClosest = debugClosestSlot();

        // finalize hits when a slot rotates away
        if (lastClosestSlot != null && currentClosest != lastClosestSlot) {
            finalizeSlot(lastClosestSlot); //clears hits for previous slot
        }
        lastClosestSlot = currentClosest;

        updateSlotClassification(currentClosest); // pass current slot for telemetry
    }

    //slot geometry
    private double getSlotCenterAngle(IndexerState s) {
        double angle = s.index * SLOT_SPACING_DEG;
        angle += offsetAngle;

        if (!intaking) {
            angle += outtakeOffsetAngle; // NOT 180 unless you confirm mechanically
        }

        return mod(angle, 360.0);
    }

    private double angleError(double a, double b) {
        return Math.abs(mod(a - b + 180.0, 360.0) - 180.0);
    }

    // classification

    private void updateSlotClassification(IndexerState currentSlot) {
        double current = getMeasuredAngle();

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(current, getSlotCenterAngle(s));

            if (err <= SLOT_ASSIGN_TOLERANCE) {
                SlotObservation obs = observations[s.index];

                ArtifactColor instantColor =
                        colorSensor.hasArtifact()
                                ? colorSensor.classifyColorOnly()
                                : ArtifactColor.EMPTY;

                int totalBefore = obs.totalHits();
                if (totalBefore >= 8) {
                    double emptyPct = obs.emptyHits / (double) totalBefore;
                    if (emptyPct >= 0.85 && instantColor != ArtifactColor.EMPTY) {
                        obs.reset();
                    }
                }

                obs.record(instantColor);

                // Recompute AFTER record
                int total = obs.totalHits();

                artifacts[s.index] = obs.resolveWithThreshold(
                        GREEN_THRESHOLD,
                        PURPLE_THRESHOLD,
                        EMPTY_THRESHOLD,
                        UNKNOWN_THRESHOLD
                );

                if (telemetry != null && s == currentSlot) {
                    telemetry.addData("Slot " + s + " hit %",
                            String.format(
                                    "G: %.0f%%, P: %.0f%%, E: %.0f%%, U: %.0f%%",
                                    total > 0 ? obs.greenHits * 100.0 / total : 0,
                                    total > 0 ? obs.purpleHits * 100.0 / total : 0,
                                    total > 0 ? obs.emptyHits * 100.0 / total : 0,
                                    total > 0 ? obs.unknownHits * 100.0 / total : 0
                            ));

                    colorSensor.addTelemetry(telemetry);
                }


            }
        }
    }

    private void finalizeSlot(IndexerState s) {
        //resolve final color
        artifacts[s.index] = observations[s.index].resolveWithThreshold(
                GREEN_THRESHOLD,
                PURPLE_THRESHOLD,
                EMPTY_THRESHOLD,
                UNKNOWN_THRESHOLD
        );

        // Reset all hits for the slot
        observations[s.index].reset();
    }



    //debug

    public IndexerState debugClosestSlot() {
        double current = getMeasuredAngle();

        IndexerState best = null;
        double bestErr = Double.MAX_VALUE;

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(current, getSlotCenterAngle(s));
            if (err < bestErr) {
                bestErr = err;
                best = s;
            }
        }
        return best;
    }

    public double debugClosestSlotErrorDeg() {
        IndexerState s = debugClosestSlot();
        return angleError(getMeasuredAngle(), getSlotCenterAngle(s));
    }

    public double debugSlotErrorDeg(IndexerState s) {
        return angleError(getMeasuredAngle(), getSlotCenterAngle(s));
    }

    public boolean debugSlotIsOverSensor(IndexerState s) {
        return debugSlotErrorDeg(s) <= SLOT_ASSIGN_TOLERANCE;
    }

    public String debugAssignmentReason() {
        IndexerState s = debugClosestSlot();
        double err = debugClosestSlotErrorDeg();

        if (err > SLOT_ASSIGN_TOLERANCE) {
            return "Between slots (err=" + String.format("%.1f", err) + "dergees)";
        }
        return "Aligned with " + s;
    }

    public double debugSecondClosestSlotErrorDeg() {
        double current = getMeasuredAngle();
        double best = Double.MAX_VALUE;
        double second = Double.MAX_VALUE;

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(current, getSlotCenterAngle(s));
            if (err < best) {
                second = best;
                best = err;
            } else if (err < second) {
                second = err;
            }
        }
        return second;
    }

    public double getVoltage()
    {
        return servoControl.getVoltage();
    }

    public double getTargetVoltage()
    {
        return servoControl.getTargetVoltage();
    }

    /* =========================
       UTIL
       ========================= */

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double mod(double v, double m) {
        double r = v % m;
        return r < 0 ? r + m : r;
    }

    private static class SlotObservation {
        int greenHits = 0;
        int purpleHits = 0;
        int emptyHits = 0;
        int unknownHits = 0;

        void reset() {
            greenHits = 0;
            purpleHits = 0;
            emptyHits = 0;
            unknownHits = 0;
        }

        void record(ArtifactColor c) {
            switch (c) {
                case GREEN:
                    greenHits++;
                    break;
                case PURPLE:
                    purpleHits++;
                    break;
                case EMPTY:
                    emptyHits++;
                    break;
                case UNKNOWN:
                    unknownHits++;
                    break;
            }
        }

        int totalHits() {
            return greenHits + purpleHits + emptyHits + unknownHits;
        }

        ArtifactColor resolveWithThreshold(double greenThresh, double purpleThresh, double emptyThresh, double unknownThresh) {
            int total = totalHits();
            if (total == 0) return ArtifactColor.EMPTY;

            if (greenHits / (double) total >= greenThresh) return ArtifactColor.GREEN;
            if (purpleHits / (double) total >= purpleThresh) return ArtifactColor.PURPLE;
            if (emptyHits / (double) total >= emptyThresh) return ArtifactColor.EMPTY;
            if (unknownHits / (double) total >= unknownThresh) return ArtifactColor.UNKNOWN;

            return ArtifactColor.UNKNOWN; // fallback if nothing crosses threshold
        }
    }

}
