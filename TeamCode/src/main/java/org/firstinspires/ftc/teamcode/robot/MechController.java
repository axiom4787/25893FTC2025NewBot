package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.VisionController;


public class MechController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private final VisionController visionController;
    private MechState currentState;

    // Hardware constants
    private static final double MAX_SERVO_ROTATION = 300.0; // degrees

    // Limit constants
    private double lastIndexer = 1;
    // Offset constants
    private static final double INDEXER_OFFSET = 150;

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

            case SHOOTING:
                setIndexer(0);
                currentState = MechState.SHOOTING;
                break;

            case INTAKE:
                setIndexer(0);
                currentState = MechState.INTAKE;
                break;

            case PURPLE_SORTING:
                setIndexer(0);
                currentState = MechState.PURPLE_SORTING;
                break;

            case GREEN_SORTING:
                setIndexer(0);
                currentState = MechState.GREEN_SORTING;
                break;

            case APRIL_TAG:
                setIndexer(0);
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
            double pos = (targetDegrees + INDEXER_OFFSET) / MAX_SERVO_ROTATION;
            pos = Math.max(0, Math.min(1, pos));
            robot.indexer.setPosition(pos);
            lastIndexer = targetDegrees;
        }
    }

    // Status
    public double statusIndexer(){
        return (robot.indexer.getPosition() * MAX_SERVO_ROTATION) - INDEXER_OFFSET;
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