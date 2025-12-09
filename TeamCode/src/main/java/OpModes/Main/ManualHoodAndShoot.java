package OpModes.Main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import OpModes.Main.Components.LauncherComponent;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name="Hood and Shoot Control", group="Individual Test")
public class ManualHoodAndShoot extends OpMode {

    private LauncherComponent launcherComponent;
    private Limelight3A limelight;
    
    // Odometry tracking via IMU with odometry pods
    private GoBildaPinpointDriver imu;
    
    private double odoX = 0.0; // Starting at 0, 0
    private double odoY = 0.0;
    
    // Limelight constants (matching AutoHoodAndShoot)
    private static final double APRILTAG_REAL_HEIGHT_METERS = 0.2032; // 8 inches before now i made it 30
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 49.5;   // Limelight 3A vertical FOV
    private static final int IMAGE_WIDTH_PIXELS = 1280;
    private static final int IMAGE_HEIGHT_PIXELS = 720;
    
    private double flywheelPower = 0.1; // starting power
    private boolean spinning = false;   // flywheel state
    private final double HOOD_INCREMENT = 0.05;
    
    // Edge detection for button debouncing
    private boolean lastTriangle = false;
    private boolean lastCross = false;
    private boolean lastSquare = false;
    private boolean lastCircle = false;

    @Override
    public void init() {
        launcherComponent = new LauncherComponent();
        launcherComponent.initialize(hardwareMap, telemetry);
        launcherComponent.setPower(flywheelPower);
        
        // Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.start();
        }
        
        // Initialize odometry pods via IMU
        try {
            imu = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
            if (imu != null) {
                // Configure odometry pods (you may need to adjust these values)
                // imu.setOffsets(xPodOffset, yPodOffset, DistanceUnit.INCH);
                // imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                // imu.setEncoderDirections(...);
                
                // Set initial position to (0, 0, 0)
                imu.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                telemetry.addData("Odometry", "Initialized - Starting at (0, 0)");
            } else {
                telemetry.addData("Odometry", "ERROR: IMU not found!");
            }
        } catch (Exception e) {
            telemetry.addData("Odometry", "ERROR: " + e.getMessage());
            imu = null;
        }
        
        telemetry.addData("Status", "Initialized. Flywheel power: " + flywheelPower);
        telemetry.addData("Status", "Initialized. Servo position: %.2f", launcherComponent.getHoodPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update odometry from IMU with odometry pods
        if (imu != null) {
            imu.update();
            Pose2D pose = imu.getPosition();
            odoX = pose.getX(DistanceUnit.INCH);
            odoY = pose.getY(DistanceUnit.INCH);
        }
        
        // Calculate limelight distance (using TA method from AutoHoodAndShoot)
        double limelightDistance = -1.0;
        if (limelight != null && limelight.isConnected()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double taPercent = result.getTa();
                if (taPercent > 0.0) {
                    double pixelArea = (taPercent / 100.0) * (IMAGE_WIDTH_PIXELS * IMAGE_HEIGHT_PIXELS);
                    double tagPixelHeight = Math.sqrt(pixelArea);
                    double focalPx = (IMAGE_HEIGHT_PIXELS / 2.0)
                            / Math.tan(Math.toRadians(CAMERA_VERTICAL_FOV_DEGREES / 2.0));

                    double distanceMeters = (APRILTAG_REAL_HEIGHT_METERS * focalPx) / tagPixelHeight;
                    limelightDistance = distanceMeters * 3.28084; // Convert to feet
                }
            }
        }
        
        // Increase power with Triangle (Y) - edge detection
        if (gamepad1.triangle && !lastTriangle) {
            flywheelPower += 0.05;
            flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));
            launcherComponent.setPower(flywheelPower);
        }

        // Decrease power with X (Cross) - edge detection
        if (gamepad1.cross && !lastCross) {
            flywheelPower -= 0.05;
            flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));
            launcherComponent.setPower(flywheelPower);
        }

        // Start spinning with Square - edge detection
        if (gamepad1.square && !lastSquare) {
            spinning = true;
            launcherComponent.setSpinning(true);
        }

        // Stop spinning with Circle - edge detection
        if (gamepad1.circle && !lastCircle) {
            spinning = false;
            launcherComponent.setSpinning(false);
        }

        // Update launcher (flywheel)
        launcherComponent.update();

        // Hood control - Move servo backward on Cross
        if (gamepad1.cross && !lastCross) {
            launcherComponent.decrementHood();
        }

        // Move servo forward on Triangle (Y)
        if (gamepad1.triangle && !lastTriangle) {
            launcherComponent.incrementHood();
        }

        // Update edge detection states
        lastTriangle = gamepad1.triangle;
        lastCross = gamepad1.cross;
        lastSquare = gamepad1.square;
        lastCircle = gamepad1.circle;

        // Telemetry - Display all data for prediction model
        telemetry.addLine("=== PREDICTION MODEL DATA ===");
        telemetry.addData("Odo X (in)", "%.4f", odoX);
        telemetry.addData("Odo Y (in)", "%.4f", odoY);
        telemetry.addData("Limelight Distance (ft)", limelightDistance > 0 ? String.format("%.4f", limelightDistance) : "N/A");
        telemetry.addData("Hood Pos", "%.4f", launcherComponent.getHoodPosition());
        telemetry.addData("Flywheel Speed", "%.4f", flywheelPower);
        telemetry.addLine("");
        telemetry.addLine("=== CONTROL STATUS ===");
        telemetry.addData("Status", spinning ? "Shooting" : "Stopped");
        telemetry.update();
    }
    
    @Override
    public void stop() {
        // Stop limelight
        if (limelight != null) {
            limelight.stop();
        }
    }
}
