package OpModes.Main.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class SpindexerComponent {
    private LinearOpMode opMode;
    private Telemetry telemetry;

    // Hardware
    private NormalizedColorSensor intakeColorSensor;
    private Servo indexServo;
    private CRServo intakeServo;
    private CRServo intakeServo2;
    private Servo kickerServo;

    // State
    private int xPressCount = 0;  // counts how many balls have been shot
    private Map<Integer, String> indexColors = new HashMap<>();
    private String[] need_colors = {"purple", "purple", "green"};
    private int flag = 0;
    private int currentDivision = 0;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private double targetDegrees = 0.0;
    private boolean intakeOn = false;

    // Constants
    public static final double MAX_DEGREES = 720.0;
    public static final double DIVISION_DEGREES = 57.0;
    public static final double SPEED_DEG_PER_STEP = 1.65;
    public static final double INITIAL_POSITION_OFFSET = 35.0;
    public static final double KICKER_RESET_POSITION = 0.89;
    public static final double KICKER_FLICK_POSITION = 0.98;
    public static final double SHOOTING_DEGREE_ADJUSTMENT = 36.0;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        indexServo = hardwareMap.get(Servo.class, "indexServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        kickerServo = hardwareMap.get(Servo.class, "kicker");

        indexColors.put(0, "none");
        indexColors.put(1, "none");
        indexColors.put(2, "none");

        // Initialize exactly as you needed earlier:
        targetDegrees = MAX_DEGREES - INITIAL_POSITION_OFFSET;
        indexServo.setPosition(posFromDeg(targetDegrees));

        telemetry.addLine("Ready. Press A to move 60°.");
        addTelemetry();
        telemetry.update();
        kickerServo.setPosition(KICKER_RESET_POSITION);
    }

    public void update(boolean gamepadA, boolean gamepadB, boolean gamepadX, boolean gamepadY) {
        // ---------------- COLOR DETECTION -----------------
        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        float hue = JavaUtil.colorToHue(colors.toColor());
        telemetry.addData("Hue", hue);
        boolean found = false;
        String detectedColor = "unknown";

        if (hue > 160 && hue < 350) {
            detectedColor = "purple";
            found = true;
        } else if (hue >= 100 && hue <= 160) {
            detectedColor = "green";
            found = true;
        }

        if (found) {
            indexColors.put(currentDivision, detectedColor);
            telemetry.addData("Detected new ball:", detectedColor);
            try {
                // Pause execution for 3 seconds (3000 milliseconds)
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                // Handle the InterruptedException if the thread is interrupted while sleeping
                e.printStackTrace();
            }
            rotateOneDivision();
        }

        // ---------------- BUTTON Y -------------------------
        if (gamepadY && !prevY) {
            if (intakeOn) {
                intakeOn = false;
                intakeServo.setPower(0);
            } else {
                intakeOn = true;
                intakeServo.setPower(1.0);
            }
        }
        prevY = gamepadY;

        // ---------------- BUTTON A -------------------------
        if (gamepadA && !prevA) {
            targetDegrees = clipDeg(targetDegrees - DIVISION_DEGREES);
            smoothMoveTo(targetDegrees);
            currentDivision = (currentDivision + 1) % 3;
        }
        prevA = gamepadA;

        // ---------------- BUTTON B -------------------------
        if (gamepadB && !prevB) {
            targetDegrees = MAX_DEGREES - INITIAL_POSITION_OFFSET;
            smoothMoveTo(MAX_DEGREES - INITIAL_POSITION_OFFSET);
            currentDivision = 0;
        }
        prevB = gamepadB;

        // ---------------- BUTTON X -------------------------
        if (gamepadX && !prevX) {
            // Shooting sequence handled by shootBall method
            shootBall();
        }
        prevX = gamepadX;

        addTelemetry();
    }

    public void shootBall() {
        // Find the division of the next needed color
        int goalDiv = findDivisionWithColor(need_colors[(flag) % need_colors.length]); // add degree change
        if (goalDiv > 2) {
            goalDiv -= 3;
        }
        moveToDivision(goalDiv);
        rotateOneDivision();
        //rotateOneDivision();
        targetDegrees -= SHOOTING_DEGREE_ADJUSTMENT;
        smoothMoveTo(targetDegrees);
        indexColors.put(flag, "none");
        //kickerWheelServo.setPower(1.0); //Run kicker wheel servo

        try {
            // Pause execution for 1.3 seconds (1500 milliseconds)
            Thread.sleep(1300);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            e.printStackTrace();
        }

        kickerServo.setPosition(KICKER_FLICK_POSITION); //Flick up kicker servo
        try {
            // Pause execution for 1 seconds (1000 milliseconds)
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            e.printStackTrace();
        }
        kickerServo.setPosition(KICKER_RESET_POSITION);

        try {
            // Pause execution for 1 seconds (1000 milliseconds)
            Thread.sleep(1300);
        } catch (InterruptedException e) {
            // Handle the InterruptedException if the thread is interrupted while sleeping
            e.printStackTrace();
        }
        targetDegrees += SHOOTING_DEGREE_ADJUSTMENT;
        smoothMoveTo(targetDegrees);
        //reset kickerServo position
        //kickerWheelServo.setPower(0); // set kicker wheel servo back to zero

        telemetry.addLine("Ball shot!");
        // "Shoot" the ball
        xPressCount++;
        flag++; // advance to next needed color

        // After 3 shots, reset everything
        if (xPressCount >= 3) {
            //STOP SHOOTER - handled by FlywheelComponent
            // Reset HashMap
            indexColors.put(0, "none");
            indexColors.put(1, "none");
            indexColors.put(2, "none");

            // Reset servo
            targetDegrees = MAX_DEGREES - INITIAL_POSITION_OFFSET;
            smoothMoveTo(targetDegrees);

            // Reset counters
            currentDivision = 0;
            xPressCount = 0;
            flag = 0;
        }
    }

    public boolean shouldStopFlywheel() {
        return xPressCount >= 3;
    }

    public boolean isPrevX() {
        return prevX;
    }

    // Telemetry helper
    public void addTelemetry() {
        telemetry.addData("Target°", "%.1f", targetDegrees);
        telemetry.addData("Servo Pos", "%.3f", indexServo.getPosition());
        telemetry.addData("Division", currentDivision);

        telemetry.addData("Div0", indexColors.get(0));
        telemetry.addData("Div1", indexColors.get(1));
        telemetry.addData("Div2", indexColors.get(2));
        telemetry.addData("Need Next", need_colors[flag]);

        telemetry.addLine("A = Move 1 division");
        telemetry.addLine("B = Reset");
    }

    // ---------------- SMOOTH SERVO MOVEMENT (FIXED) -------------------
    public void smoothMoveTo(double newTargetDeg) {
        // ❗️THIS FIXES THE ISSUE — start from actual servo output
        double startPos = indexServo.getPosition();  // 0.0–1.0
        double startDeg = startPos * MAX_DEGREES;

        double distance = newTargetDeg - startDeg;
        int steps = (int) Math.ceil(Math.abs(distance) / SPEED_DEG_PER_STEP);

        for (int i = 1; i <= steps; i++) {
            double interpDeg = startDeg + distance * (i / (double) steps);
            indexServo.setPosition(posFromDeg(interpDeg));
            opMode.sleep(10);
        }

        indexServo.setPosition(posFromDeg(newTargetDeg));
    }

    // ---------------- DIVISION ROTATION ---------------------
    public void rotateOneDivision() {
        currentDivision = (currentDivision + 1) % 3;
        targetDegrees = clipDeg(targetDegrees - DIVISION_DEGREES);
        smoothMoveTo(targetDegrees);
    }

    public void moveToDivision(int targetDiv) {
        int steps = (targetDiv - currentDivision + 3) % 3;
        for (int i = 0; i < steps; i++) {
            rotateOneDivision();
            opMode.sleep(150);
        }
    }

    public int findDivisionWithColor(String c) {
        for (int i = 0; i < 3; i++) {
            if (indexColors.get(i).equals(c)) return i;
        }
        return currentDivision;
    }

    // ---------------- DEG <-> POS HELPERS ---------------------
    public static double posFromDeg(double degrees) {
        return Range.clip(degrees / MAX_DEGREES, 0.0, 1.0);
    }

    public static double clipDeg(double d) {
        return Range.clip(d, 0.0, MAX_DEGREES);
    }
}

