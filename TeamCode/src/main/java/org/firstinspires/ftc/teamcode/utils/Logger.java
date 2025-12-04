package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.*;

public class Logger {

    public enum LoggerMode {
        DETAILED(0),
        CRITICAL(1),
        STATUS(2);

        private final int rank;

        LoggerMode(int rank) {
            this.rank = rank;
        }

        public boolean includes(LoggerMode other) {
            return other.rank >= this.rank;
        }
    }

    private LoggerMode currentMode;
    private final Telemetry telemetry;

    // persistent lines
    private final List<String> history = new ArrayList<>();

    // overwriting key-values
    private final Map<String, Object> fields = new HashMap<>();

    public Logger(LoggerMode mode, Telemetry telemetry) {
        this.currentMode = mode;
        this.telemetry = telemetry;
    }

    public void setMode(LoggerMode mode) {
        this.currentMode = mode;
    }

    // persistent log
    public void log(LoggerMode level, Object... messages) {
        if (!currentMode.includes(level)) return;

        StringBuilder sb = new StringBuilder();
        for (Object msg : messages) sb.append(msg);

        String finalMessage = sb.toString();
        System.out.println(finalMessage);

        // store permanently
        history.add(finalMessage);
    }

    // overwriting log
    public void logData(LoggerMode level, String field, Object data) {
        if (!currentMode.includes(level)) return;

        System.out.println(field + ": " + data);

        // store current value (overwrites old)
        fields.put(field, data);
    }

    public void update() {
        // full clear
        telemetry.clearAll();

        // re-render persistent logs
        for (String line : history) {
            telemetry.addLine(line);
        }

        // re-render overwriting fields
        for (Map.Entry<String, Object> e : fields.entrySet()) {
            telemetry.addData(e.getKey(), e.getValue());
        }

        telemetry.update();
    }
}
