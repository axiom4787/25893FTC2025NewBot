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
    public static final double[] INTAKE = {0, 120, 240}; // Indexer 0, 1, 2 @ Intake Post degrees
    public static final double[] SHOOT = {150, 270, 30}; // Indexer 0, 1, 2 @ Shooting Post degrees
    private static final double MAX_SERVO_ROTATION = 270.0; // Degrees
    public static final double INTAKE_TICKS_PER_FULL_ROTATION = 537.7;//Encoder Resolution PPR for RPM 312
    private final long POST_ROTATE_WAIT_MS = 1000;     // After every rotation
    private final long MIN_INTAKE_SPIN_MS = 300;       // Minimum spin before detecting
    private final long MOTOR_WAIT_MS = 3000; // 3 seconds for Shooting motor to reach full speed
    private final long POST_INDEXER_WAIT_MS = 1000; // 1 second post Indexer rotation
    private final long LIFT_WAIT_MS = 2000; // 2 seconds for Lifter in Up position for shooting
    private final long DROP_WAIT_MS = 1000; // 1 second post Lifter in Down position
    private final long APRIL_TAG_WAIT_MS = 3000; // 3 seconds waiting to detect AprilTag


    // Limit constants
    public static final int lifterDown = 21; // Lifter down angle degrees
    public static final int lifterUp = 56; // Lifter up angle degrees

    // Offset constants

    // Variables
    public int[] tagPattern = {0, 0, 0, 0}; // Tag ID & Pattern
    public int[] indexer = {2, 1, 1}; // GPP - Color of artifact in Indexer 0, 1, 2
    private int artifactCount = 3;
    private double lastIndexer = 1;
    private int lastLifter = 0;
    private boolean firstIntake = true;
    private int intakeTargetIndex = -1;
    private int intakeStage = 0;
    private long intakeStageStart = 0;
    private boolean shootingMotorRunning = false;
    private boolean motorInitialWaitDone = false;
    private int shootPatternIndex = 1;
    private int slotToShoot = -1;
    private long shootStageStart = 0;
    private long shootElapsed = 0;
    private int shootStage = 0;
    private long aprilTagStageStart = 0;
    private boolean aprilTagRunning = false;
    private long aprilTagElapsed = 0;
    private boolean humanIntakeRunning = false;
    private int humanIndex = -1;

    // Constructor
    public MechController(RobotHardware RoboRoar, VisionController visionController) {
        this.robot = RoboRoar;
        this.telemetry = RoboRoar.telemetry;
        this.visionController = visionController;
        this.currentState = MechState.IDLE;
    }

    // State machine handler
    public void handleMechState(MechState state) {
        switch (state) {
            case START:
                currentState = MechState.START;
                setLifter(0);
                setIndexer(0);
                handleMechState(MechState.IDLE);
                currentState = MechState.IDLE;
                break;

            case IDLE:
                currentState = MechState.IDLE;
                break;

            case SHOOT_STATE:
                currentState = MechState.SHOOT_STATE;
                if (!shootingMotorRunning) { // Start shooting Motor
                    runShootingMot(1);
                    shootingMotorRunning = true;
                    if (!motorInitialWaitDone) {
                        shootStageStart = System.currentTimeMillis();
                        shootStage = -1;
                    }
                }

                if (shootPatternIndex >= tagPattern.length || artifactCount <= 0) {
                    runShootingMot(0); // Stop shooting stage
                    shootingMotorRunning = false;
                    motorInitialWaitDone = false;
                    shootStage = 0;
                    shootPatternIndex = 1;
                    slotToShoot = -1;
                    shootElapsed = 0;
                    currentState = MechState.IDLE;
                    break;
                }

                // Shooting stage machine
                switch (shootStage) {
                    case -1:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= MOTOR_WAIT_MS) { // Waiting for shooting motor to reach full speed
                            motorInitialWaitDone = true;
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 0;
                        }
                        break;

                    case 0:
                        if (slotToShoot == -1) {
                            int targetColor = tagPattern[shootPatternIndex]; // Checking motif pattern color to shoot
                            for (int i = 0; i < indexer.length; i++) {
                                if (indexer[i] == targetColor) { // Finding the color to shoot in index
                                    slotToShoot = i;
                                    break;
                                }
                            }
                            if (slotToShoot != -1) {
                                setIndexer(SHOOT[slotToShoot]); // Setting indexer to shooting position
                                shootStageStart = System.currentTimeMillis();
                                shootStage = 3;
                            } else {
                                shootPatternIndex++;
                            }
                        }
                        break;

                    case 3:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= POST_INDEXER_WAIT_MS) { // Waiting post indexer rotation
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 1;
                        }
                        break;

                    case 1:
                        setLifter(1); // Lifter up
                        shootStageStart = System.currentTimeMillis();
                        shootStage = 2;
                        break;

                    case 2:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= LIFT_WAIT_MS) {
                            setLifter(0); // Lifter down
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 4;
                        }
                        break;

                    case 4:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= DROP_WAIT_MS) {
                            int targetColor = tagPattern[shootPatternIndex];
                            for (int i = 0; i < indexer.length; i++) {
                                if (indexer[i] == targetColor) {
                                    indexer[i] = 0; // Setting 0 in indexer for the artifact shot
                                    artifactCount--;
                                    break;
                                }
                            }
                            slotToShoot = -1;
                            shootPatternIndex++;
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 0;
                        }
                        break;
                }
                break;

            case INTAKE_STATE:
                currentState = MechState.INTAKE_STATE;

                switch (intakeStage) {

                    case 0: // ROTATE to next empty slot
                        intakeTargetIndex = getEmptyIndex();
                        if (intakeTargetIndex == -1) { // All slots full
                            currentState = MechState.IDLE;
                            break;
                        }

                        setIndexer(INTAKE[intakeTargetIndex]); // Rotate indexer immediately
                        intakeStageStart = System.currentTimeMillis();
                        intakeStage = 1; // Move to INTAKE stage
                        break;

                    case 1: // INTAKE motor running
                        if (System.currentTimeMillis() - intakeStageStart < POST_ROTATE_WAIT_MS)
                            break; // Wait before starting intake

                        runIntakeMot(1); // Start intake
                        intakeStageStart = System.currentTimeMillis();
                        intakeStage = 2; // Check for artifact
                        break;

                    case 2: // Monitor intake
                        int color = visionController.artifactColor(); // Detect artifact
                        boolean detected = color != 0;

                        if (detected) {
                            runIntakeMot(0); // Stop motor
                            indexer[intakeTargetIndex] = color; // Store artifact
                            artifactCount++;
                            intakeStageStart = System.currentTimeMillis(); // Begin POST_WAIT
                            intakeStage = 3;
                        }
                        break;

                    case 3: // POST_WAIT after intake
                        if (System.currentTimeMillis() - intakeStageStart >= POST_ROTATE_WAIT_MS) {
                            intakeStage = 0; // Loop back to ROTATE for next empty slot
                        }
                        break;
                }
                break;

            case SHOOT_PURPLE:
                currentState = MechState.SHOOT_PURPLE;
                if (!shootingMotorRunning) {
                    int purpleIndex = getPurpleIndex();
                    if (purpleIndex != -1) {
                        runShootingMot(1); // Start shooter
                        shootingMotorRunning = true;
                        setIndexer(SHOOT[purpleIndex]); // Setting indexer to shooting position
                        shootStageStart = System.currentTimeMillis();
                        shootStage = -1;
                    } else {
                        currentState = MechState.IDLE;
                        break;
                    }
                }

                switch (shootStage) {
                    case -1:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= MOTOR_WAIT_MS) {
                            motorInitialWaitDone = true;
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 1;
                        }
                        break;

                    case 1:
                        setLifter(1); // Lifter up
                        shootStageStart = System.currentTimeMillis();
                        shootStage = 2;
                        break;

                    case 2:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= LIFT_WAIT_MS) {
                            setLifter(0); // Lifter down
                            runShootingMot(0);
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 3;
                        }
                        break;

                    case 3:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= DROP_WAIT_MS) {
                            int purpleIndex = getPurpleIndex();
                            if (purpleIndex != -1) {
                                indexer[purpleIndex] = 0;
                            }
                            shootingMotorRunning = false;
                            motorInitialWaitDone = false;
                            artifactCount--;
                            shootStage = 0;
                            shootElapsed = 0;
                            currentState = MechState.IDLE;
                        }
                        break;
                }
                break;

            case SHOOT_GREEN:
                currentState = MechState.SHOOT_GREEN;
                if (!shootingMotorRunning) {
                    int greenIndex = getGreenIndex();
                    if (greenIndex != -1) {
                        runShootingMot(1); // Start shooter
                        shootingMotorRunning = true;
                        setIndexer(SHOOT[greenIndex]); // Setting indexer to shooting position
                        shootStageStart = System.currentTimeMillis();
                        shootStage = -1;
                    } else {
                        currentState = MechState.IDLE;
                        break;
                    }
                }

                switch (shootStage) {
                    case -1:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= MOTOR_WAIT_MS) {
                            motorInitialWaitDone = true;
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 1;
                        }
                        break;

                    case 1:
                        setLifter(1); // Lifter up
                        shootStageStart = System.currentTimeMillis();
                        shootStage = 2;
                        break;

                    case 2:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= LIFT_WAIT_MS) {
                            setLifter(0); // Lifter down
                            runShootingMot(0);
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 3;
                        }
                        break;

                    case 3:
                        shootElapsed = System.currentTimeMillis() - shootStageStart;
                        if (shootElapsed >= DROP_WAIT_MS) {
                            int greenIndex = getGreenIndex();
                            if (greenIndex != -1) {
                                indexer[greenIndex] = 0;
                            }
                            shootingMotorRunning = false;
                            motorInitialWaitDone = false;
                            artifactCount--;
                            shootStage = 0;
                            shootElapsed = 0;
                            currentState = MechState.IDLE;
                        }
                        break;
                }
                break;

            case APRIL_TAG:
                currentState = MechState.APRIL_TAG;
                tagPattern = visionController.findTagPattern(visionController.getAprilTag());
                if (!aprilTagRunning) {
                    aprilTagStageStart = System.currentTimeMillis();
                    aprilTagRunning = true;
                } else {
                    aprilTagElapsed = System.currentTimeMillis() - aprilTagStageStart;
                    if (tagPattern[0] == 0 || aprilTagElapsed >= APRIL_TAG_WAIT_MS) { // If no tag detected or timeout reached
                        tagPattern = visionController.findTagPattern(visionController.getAprilTag());
                        if (tagPattern[0] == 0) { // If still no tag detected
                            tagPattern = new int[]{21, 2, 1, 1};  // ID 21: GPP
                        }
                    }
                    handleMechState(MechState.START);
                    currentState = MechState.START;; // Stop AprilTag stage
                    aprilTagElapsed = 0;
                    aprilTagRunning = false;
                    aprilTagStageStart = 0;
                }
                break;

            case HUMAN_STATE:
                currentState = MechState.HUMAN_STATE;
                if (!humanIntakeRunning) {
                    humanIndex = getEmptyIndex(); // Finding slot 0 for intake
                    if (humanIndex != -1) { // Checking if all slots are full
                        setIndexer(INTAKE[humanIndex]); // Setting indexer to intake position
                        humanIntakeRunning = true;
                    } else {
                        currentState = MechState.IDLE; // Stop human stage
                        break;
                    }
                } else {
                    int color = visionController.artifactColor(); // Reading sensor color
                    if (color != 0) { // Checking if Purple or Green artifact
                            indexer[humanIndex] = color; // Storing the artifact color based on indexer slot filled
                            artifactCount++;
                            humanIntakeRunning = false;
                        }
                    }
                break;
        }
    }

    // State machine methods

    public MechState getCurrentState() {
        return currentState;
    }

    public void setState(MechState newState) {
        currentState = newState;
    }
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
            double currentPos = robot.intakeMot.getCurrentPosition();
            int targetPos = (int)(((Math.ceil(currentPos/INTAKE_TICKS_PER_FULL_ROTATION))+1)*INTAKE_TICKS_PER_FULL_ROTATION);
            robot.intakeMot.setTargetPosition(targetPos);
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
    public int getEmptyIndex() {
        for (int i = 0; i < indexer.length; i++) {
            if (indexer[i] == 0) {
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
    public int getGreenIndex() {
        for (int i = 0; i < indexer.length; i++) {
            if (indexer[i] == 2) {
                return i;
            }
        }
        return -1;
    }

    // Status
    public double statusIndexer(){
        return robot.indexer.getPosition() * MAX_SERVO_ROTATION;
    }
    public double statusLifter(){
        return robot.lifter.getPosition() * MAX_SERVO_ROTATION;
    }

    // Telemetry output
    public void allTelemetry() {
        telemetry.addData("State", currentState);

        telemetry.addData("Tag Pattern",
                "%d --> %d | %d | %d",
                tagPattern[0], tagPattern[1], tagPattern[2], tagPattern[3]);

        telemetry.addData("Artifact Count --> Indexer",
                "%d --> %d | %d | %d",
                artifactCount, indexer[0], indexer[1], indexer[2]);

        if (robot.pinpoint != null) {
            telemetry.addData("Pinpoint",
                    "X: %.1f in | Y: %.1f in | Heading: %.1f°",
                    robot.pinpoint.getPosX(DistanceUnit.INCH),
                    robot.pinpoint.getPosY(DistanceUnit.INCH),
                    robot.pinpoint.getHeading(AngleUnit.DEGREES)
            );
            robot.pinpoint.update();
        }

        telemetry.addData("Indexer | Lifter",
                "%.1f° | %.1f°", statusIndexer(), statusLifter());

        visionController.sensorTelemetry();
        //visionController.aprilTagTelemetry();

        telemetry.update();
    }

}