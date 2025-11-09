package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MechController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private final VisionController visionController;
    private MechState currentState;

    // Hardware constants
    private static final double MAX_SERVO_ROTATION = 300.0; // Degrees
    public static final double INTAKE_TICKS_PER_FULL_ROTATION = 537.7;// Encoder Resolution PPR for RPM 312
    public static final double[] INTAKE= {0, 120, 240}; // Indexer 0, 1, 2 @ Intake Post degrees
    public static final double[] SHOOT = {180, 300, 60}; // Indexer 0, 1, 2 @ Shooting Post degrees

    // Limit constants
    public static final int lifterDown = 21;
    public static final int lifterUp = 56;


    // Offset constants

    // Variables
    public int[] tagPattern = {0, 0, 0, 0}; // Tag ID & Pattern
    public int[] indexer = {2, 1, 1}; // GPP - Color of artifact in Indexer 0, 1, 2
    private double lastIndexer = 1;
    private double lastLifter = 0;

    // Constructor
    public MechController(RobotHardware RoboRoar) {
        this.robot = RoboRoar;
        this.telemetry = RoboRoar.telemetry;
        this.visionController = new VisionController(RoboRoar);
        this.currentState = MechState.IDLE;
    }

    // State machine handler
    public void handleMechState(MechState state) {
        switch (state) {
            case IDLE:
                setIndexer(0);
                currentState = MechState.IDLE;
                break;

            case SHOOT_STATE:
                setIndexer(0);
                currentState = MechState.SHOOT_STATE;
                break;

            case INTAKE_STATE:
                setIndexer(0);
                currentState = MechState.INTAKE_STATE;
                break;

            case SORT_PURPLE:
                setIndexer(0);
                currentState = MechState.SORT_PURPLE;
                break;

            case SORT_GREEN:
                setIndexer(0);
                currentState = MechState.SORT_GREEN;
                break;

            case APRIL_TAG:
                tagPattern = visionController.findTagPattern(visionController.getAprilTag());
                currentState = MechState.APRIL_TAG;
                break;

            case HUMAN_STATE:
                setIndexer(0);
                currentState = MechState.HUMAN_STATE;
                break;
        }
    }

    // State machine methods
    public void setIndexer(double targetDegrees) {
        if (lastIndexer != targetDegrees) {
            double pos = targetDegrees / MAX_SERVO_ROTATION;
            pos = Math.max(0, Math.min(1, pos));
            robot.indexer.setPosition(pos);
            lastIndexer = targetDegrees;
        }
    }

    public void runIntakeMot(double power) {
        if (Math.abs(power) > 0.01) {
            robot.intakeMot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.intakeMot.setPower(power);
        } else {
            robot.intakeMot.setPower(0);
            robot.intakeMot.setTargetPosition(0);
            robot.intakeMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeMot.setPower(0.5);
        }
    }

    public void runShootingMot(double power) {
        robot.shootingMot.setPower(power);
    }

    public void setLifter(int down0up1) {
        if (lastLifter != down0up1) {
            if (down0up1 == 1) {
                robot.lifter.setPosition(lifterUp / MAX_SERVO_ROTATION);
                lastLifter = 1;
            } else {
                robot.lifter.setPosition(lifterDown / MAX_SERVO_ROTATION);
                lastLifter = 0;
            }
        }
    }
    public double getEmptyIndex() {
        int zeroPos = getZeroPos();
        if (zeroPos != -1) {
            return INTAKE[zeroPos];
        }
        return -1;
    }
    public int getZeroPos() {
        for (int i = 0; i < indexer.length; i++) {
            if (indexer[i] == 0) {
                return i;
            }
        }
        return -1;
    }

    public int getGreenIndex() {
        for (int i = 0; i < indexer.length; i++) {
            if (indexer[i] == 2) {
                return i;
            }
        }
        return -1;
    }

    public int getPurpleIndex() {
        for (int i = 0; i < indexer.length; i++) {
            if (indexer[i] == 1) {
                return i;
            }
        }
        return -1;
    }

    // Status
    public double statusIndexer(){
        return robot.indexer.getPosition() * MAX_SERVO_ROTATION;
    }
    public boolean isBusy() {
        return robot.intakeMot.isBusy() || robot.shootingMot.isBusy();
    }

    // Telemetry output
    public void allTelemetry() {
        telemetry.addData("State: ", currentState + " | Busy: " + isBusy());

        if (robot.pinpoint != null) {
            telemetry.addData("Pinpoint: ",
                    "X: %.1f in | Y: %.1f in | Heading: %.1f°",
                    robot.pinpoint.getPosX(DistanceUnit.INCH),
                    robot.pinpoint.getPosY(DistanceUnit.INCH),
                    robot.pinpoint.getHeading(AngleUnit.DEGREES)
            );
            robot.pinpoint.update();
        }
        telemetry.addData("Indexer: ", statusIndexer());
        visionController.aprilTagTelemetry();
        visionController.sensorTelemetry();
        telemetry.update();
    }
}