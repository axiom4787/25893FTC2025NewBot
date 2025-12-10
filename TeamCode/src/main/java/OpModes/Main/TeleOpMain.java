//NEED TO UPDATE FLYWHEEL SYNC AFTER KICKER MOVEMENT

package OpModes.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import OpModes.Main.Components.Turret;
import OpModes.Main.Components.Launcher;
import OpModes.Main.Components.Spindexer;
import OpModes.Main.Components.DriveTrain;
import OpModes.Main.Components.ParkingComponent;

@TeleOp(name = "TeleOpMain", group = "Linear OpMode")
public class TeleOpMain extends LinearOpMode {
    // Components
    private Turret turretComponent;
    private Launcher launcherComponent;
    private Spindexer spindexerComponent;
    private DriveTrain driveTrainComponent;
    private ParkingComponent parkingComponent;

    @Override
    public void runOpMode() {
        // Initialize components
        turretComponent = new Turret();
        turretComponent.initialize(hardwareMap, telemetry);

        launcherComponent = new Launcher();
        launcherComponent.initialize(hardwareMap, telemetry);

        spindexerComponent = new Spindexer();
        spindexerComponent.initialize(hardwareMap, telemetry, this);

        driveTrainComponent = new DriveTrain();
        driveTrainComponent.initialize(hardwareMap, telemetry);

        parkingComponent = new ParkingComponent();
        parkingComponent.initialize(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Update launcher (flywheel)
            launcherComponent.update();

            // Update turret alignment - exit OpMode if limelight not connected (matches original behavior)
            if (!turretComponent.update()) {
                return;
            }
            telemetry.update();

            // Update drive train
            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            driveTrainComponent.update(forward, right, rotate, gamepad1.cross);
            telemetry.update();

            // Update spindexer
            boolean gamepadA = gamepad1.a;
            boolean gamepadB = gamepad1.b;
            boolean gamepadX = gamepad1.x;
            boolean gamepadY = gamepad1.y;
            boolean gamepadLeftBumper = gamepad1.left_bumper;

            // Handle shooting button - start flywheel when X is pressed (before shooting sequence)
            if (gamepadX && !spindexerComponent.isPrevX()) {
                launcherComponent.setSpinning(true);
                launcherComponent.update(); // Update flywheel power immediately
            }

            spindexerComponent.update(gamepadA, gamepadB, gamepadX, gamepadY, gamepadLeftBumper);

            // Stop flywheel after 3 shots
            if (spindexerComponent.shouldStopFlywheel()) {
                launcherComponent.setSpinning(false);
            }

            // Update parking component
            parkingComponent.update(gamepad1.dpad_up, gamepad1.dpad_down);

            telemetry.update();
            idle();
        }
    }
}
