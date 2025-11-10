package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MechController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private final VisionController visionController;
    private MechState currentState;

    // Hardware constants
    public static final double[] INTAKE= {0, 120, 240}; // Indexer 0, 1, 2 @ Intake Post degrees
    public static final double[] SHOOT = {180, 300, 60}; // Indexer 0, 1, 2 @ Shooting Post degrees
    private static final double MAX_SERVO_ROTATION = 300.0; // Degrees
    private static final long INTAKE_TIMEOUT_MS = 3000; // 3 seconds max per artifact
    private final long MOTOR_WAIT_MS = 3000; // 3 seconds for Shooting motor full speed
    private final long POST_INDEXER_WAIT_MS = 1000; // 1 second post Indexer rotation
    private final long LIFT_WAIT_MS = 2000; // 2 seconds for Lifter in Up position
    private final long DROP_WAIT_MS = 1000; // 1 second post Lifter in Down position
    private final ElapsedTime timer = new ElapsedTime();


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
    private int intakeTargetIndex = -1;
    private boolean intakeRunning = false;
    private long intakeStartTime = 0;
    private boolean shootingMotorRunning = false;
    private boolean motorInitialWaitDone = false;
    private int shootPatternIndex = 1;
    private int slotToShoot = -1;
    private long shootStageStart = 0;
    private int shootStage = 0;
    private boolean delayActive = false;
    private double delayDuration = 0;

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
                currentState = MechState.IDLE;
                runShootingMot(0);
                runIntakeMot(0);
                setIndexer(0);
                setLifter(0);
                break;

            case SHOOT_STATE:
                currentState = MechState.SHOOT_STATE;
                if (!shootingMotorRunning) {
                    runShootingMot(1);
                    shootingMotorRunning = true;
                    if (!motorInitialWaitDone) {
                        shootStageStart = System.currentTimeMillis();
                        shootStage = -1;
                    }
                }

                if (shootPatternIndex >= tagPattern.length || artifactCount <= 0) {
                    runShootingMot(0);
                    shootingMotorRunning = false;
                    motorInitialWaitDone = false;
                    shootStage = 0;
                    shootPatternIndex = 1;
                    slotToShoot = -1;
                    currentState = MechState.IDLE;
                    break;
                }

                long shootElapsed = System.currentTimeMillis() - shootStageStart;

                // Shooting stage machine
                switch (shootStage) {
                    case -1:
                        if (shootElapsed >= MOTOR_WAIT_MS) {
                            motorInitialWaitDone = true;
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 0;
                        }
                        break;

                    case 0:
                        if (slotToShoot == -1) {
                            int targetColor = tagPattern[shootPatternIndex];
                            for (int i = 0; i < indexer.length; i++) {
                                if (indexer[i] == targetColor) {
                                    slotToShoot = i;
                                    break;
                                }
                            }
                            if (slotToShoot != -1) {
                                setIndexer(SHOOT[slotToShoot]);
                                shootStageStart = System.currentTimeMillis();
                                shootStage = 3;
                            } else {
                                shootPatternIndex++;
                            }
                        }
                        break;

                    case 3:
                        if (shootElapsed >= POST_INDEXER_WAIT_MS) {
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 1;
                        }
                        break;

                    case 1:
                        setLifter(1);
                        shootStageStart = System.currentTimeMillis();
                        shootStage = 2;
                        break;

                    case 2:
                        if (shootElapsed >= LIFT_WAIT_MS) {
                            setLifter(0);
                            shootStageStart = System.currentTimeMillis();
                            shootStage = 4;
                        }
                        break;

                    case 4:
                        if (shootElapsed >= DROP_WAIT_MS) {
                            int targetColor = tagPattern[shootPatternIndex];
                            for (int i = 0; i < indexer.length; i++) {
                                if (indexer[i] == targetColor) {
                                    indexer[i] = 0;
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
                if (!intakeRunning) {
                    intakeTargetIndex = getEmptyIndex();
                    if (intakeTargetIndex != -1) {
                        setIndexer(INTAKE[intakeTargetIndex]);
                        runIntakeMot(1);
                        intakeStartTime = System.currentTimeMillis();
                        intakeRunning = true;
                    } else {
                        currentState = MechState.IDLE;
                    }
                } else {
                    int color = visionController.artifactColor();
                    long intakeElapsed = System.currentTimeMillis() - intakeStartTime;
                    if (color != 0 || intakeElapsed >= INTAKE_TIMEOUT_MS) {
                        runIntakeMot(0);
                        if (color != 0 && intakeTargetIndex != -1) {
                            indexer[intakeTargetIndex] = color;
                            artifactCount++;
                        }
                        intakeRunning = false;
                    }
                }
                break;

            case SHOOT_PURPLE:
                currentState = MechState.SHOOT_PURPLE;
                setIndexer(0);
                break;

            case SHOOT_GREEN:
                currentState = MechState.SHOOT_GREEN;
                setIndexer(0);
                break;

            case APRIL_TAG:
                currentState = MechState.APRIL_TAG;
                tagPattern = visionController.findTagPattern(visionController.getAprilTag());
                break;

            case HUMAN_STATE:
                currentState = MechState.HUMAN_STATE;
                setIndexer(0);
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
    public void startDelay(double seconds) {
        delayDuration = seconds;
        timer.reset();
        delayActive = true;
    }
    public boolean isDelayActive() {
        if (delayActive && timer.seconds() >= delayDuration) {
            delayActive = false;
        }
        return delayActive;
    }

    // Status
    public double statusIndexer(){
        return robot.indexer.getPosition() * MAX_SERVO_ROTATION;
    }
    public double statusLifer(){
        return robot.lifter.getPosition() * MAX_SERVO_ROTATION;
    }

    // Telemetry output
    public void allTelemetry() {
        telemetry.addData("State: ", currentState + " | Artifact Count", artifactCount);
        if (robot.pinpoint != null) {
            telemetry.addData("Pinpoint: ",
                    "X: %.1f in | Y: %.1f in | Heading: %.1f°",
                    robot.pinpoint.getPosX(DistanceUnit.INCH),
                    robot.pinpoint.getPosY(DistanceUnit.INCH),
                    robot.pinpoint.getHeading(AngleUnit.DEGREES)
            );
            robot.pinpoint.update();
        }
        telemetry.addData("Indexer: ", statusIndexer() + " | Lifter", statusLifer());
        visionController.aprilTagTelemetry();
        visionController.sensorTelemetry();
        telemetry.update();
    }
}