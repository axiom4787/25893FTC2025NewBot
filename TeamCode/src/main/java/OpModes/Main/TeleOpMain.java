//NEED TO UPDATE FLYWHEEL SYNC AFTER KICKER MOVEMENT

package OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.HashMap;
import java.util.Map;


@TeleOp(name = "TeleOpMain", group = "Linear OpMode")
public class TeleOpMain extends LinearOpMode {
    // PRE VARIABLE SECTION

    // SPINDEXER
    int xPressCount = 0;  // counts how many balls have been shot
    public NormalizedColorSensor intakeColorSensor;
    public Servo indexServo;
    public CRServo intakeServo;
    public CRServo intakeServo2;

    public Servo kickerServo;

    //public CRServo kickerWheelServo;
    Map<Integer, String> indexColors = new HashMap<>();

    String[] need_colors = {"purple", "purple", "green"};
    int flag = 0;

    int currentDivision = 0;

    boolean prevA = false;
    boolean prevB = false;
    boolean prevX = false;

    boolean prevY = false;

    public static final double MAX_DEGREES = 720.0;
    public static final double DIVISION_DEGREES = 55.0;

    public static final double SPEED_DEG_PER_STEP = 2.0;

    public double targetDegrees = 0.0;
    boolean intakeOn = false;

    //DRIVE TRAIN

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private boolean crossPressedLast = false;


    @Override
    public void runOpMode() {
        //INIT SECTION

        //SPINDEVER INIT
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        indexServo = hardwareMap.get(Servo.class, "indexServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
        kickerServo = hardwareMap.get(Servo.class, "kicker");
        //kickerWheelServo = hardwareMap.get(CRServo.class, "kickerWheel");


        indexColors.put(0, "none");
        indexColors.put(1, "none");
        indexColors.put(2, "none");

        // Initialize exactly as you needed earlier:
        targetDegrees = MAX_DEGREES;
        indexServo.setPosition(posFromDeg(targetDegrees));

        telemetry.addLine("Ready. Press A to move 60°.");
        addTelemetry();
        telemetry.update();
        kickerServo.setPosition(0.011);

        //DRIVE TRAIN INIT
        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftfrontmotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightfrontmotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftbackmotor");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightbackmotor");

        // Correct motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            //DRIVE

            crossPressedLast = gamepad1.cross;

            // Standard mecanum driving
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Mecanum wheel math
            double frontLeftPower = forward + right + rotate;
            double frontRightPower = forward - right - rotate;
            double backLeftPower = forward - right + rotate;
            double backRightPower = forward + right - rotate;

            // Normalize
            double max = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Apply powers
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            telemetry.update();

            //SPINDEXER


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
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    // Handle the InterruptedException if the thread is interrupted while sleeping
                    e.printStackTrace();
                }
                rotateOneDivision();
            }
            // ---------------- BUTTON A -------------------------
            boolean Y = gamepad1.y;

            if (Y && !prevY){
                if (intakeOn){
                    intakeOn = false;
                    intakeServo.setPower(0);
                    intakeServo2.setPower(0);
                }
                else{
                    intakeOn = true;
                    intakeServo.setPower(1.0);
                    intakeServo2.setPower(-1.0);
                }
            }
            prevY = Y;

            boolean A = gamepad1.a;
            if (A && !prevA) {
                targetDegrees = clipDeg(targetDegrees - DIVISION_DEGREES);
                smoothMoveTo(targetDegrees);
                currentDivision = (currentDivision + 1) % 3;
            }
            prevA = A;

            // ---------------- BUTTON B -------------------------
            boolean B = gamepad1.b;
            if (B && !prevB) {
                targetDegrees = MAX_DEGREES;
                smoothMoveTo(MAX_DEGREES);
                currentDivision = 0;
            }
            prevB = B;

            // ---------------- BUTTON X -------------------------

            boolean X = gamepad1.x;
            if (X && !prevX) {
                //run shooter
                // Find the division of the next needed color

                int goalDiv = findDivisionWithColor(need_colors[(flag) % need_colors.length])+2;
                if (goalDiv > 2){
                    goalDiv -= 3;
                }
                moveToDivision(goalDiv);  // move to that division first
                indexColors.put(flag, "none");
                //kickerWheelServo.setPower(1.0); //Run kicker wheel servo

                try {
                    // Pause execution for 1.3 seconds (1500 milliseconds)
                    Thread.sleep(1300);
                } catch (InterruptedException e) {
                    // Handle the InterruptedException if the thread is interrupted while sleeping
                    e.printStackTrace();
                }





                kickerServo.setPosition(0.7); //Flick up kicker servo
                try {
                    // Pause execution for 1 seconds (1000 milliseconds)
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    // Handle the InterruptedException if the thread is interrupted while sleeping
                    e.printStackTrace();
                }
                kickerServo.setPosition(0.011); //reset kickerServo position
                //kickerWheelServo.setPower(0); // set kicker wheel servo back to zero

                telemetry.addLine("Ball shot!");
                // "Shoot" the ball
                xPressCount++;
                flag++; // advance to next needed color

                // After 3 shots, reset everything
                if (xPressCount >= 3) {
                    //STOP SHOOTER
                    // Reset HashMap
                    indexColors.put(0, "none");
                    indexColors.put(1, "none");
                    indexColors.put(2, "none");

                    // Reset servo
                    targetDegrees = MAX_DEGREES;
                    smoothMoveTo(targetDegrees);

                    // Reset counters
                    currentDivision = 0;
                    xPressCount = 0;
                    flag = 0;

                }
            }
            prevX = X;


            addTelemetry();
            telemetry.update();
            idle();

        }
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
            sleep(10);
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
            sleep(150);
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

    public void stopMotors() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

}





