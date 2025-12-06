package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="TheKeep",group="The Keep")
public class TheKeepAutotomus extends LinearOpMode {
    // Sets up the motor and servo variables and stores them as null to be used for later
    private DcMotor FrontL = null;
    private DcMotor FrontR = null;
    private DcMotor BackL = null;
    private DcMotor BackR = null;
    private DcMotor intake = null;
    private DcMotor Shooter = null;
    private Servo ballEjector = null;
    private Servo fidgetTech = null;

    // Sets up a variable to store what snap point the fidget tech is at
    private int number = 13;
    private final double[] spinPosition = {0.03,0.06,0.1,0.14,0.17,0.21,0.25,0.28,0.32,0.36,0.4,0.43,0.47,0.51,0.55,0.58,0.62,0.66,0.7,0.73,0.77,0.8,0.83,0.87,0.91,0.94,0.98};

    // Sets up the variables to store the vision and april tags camera settings
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Sets up variables to store each april tags data
    private AprilTagDetection blueBase = null;
    private boolean greenPurplePurple = false;
    private boolean purpleGreenPurple = false;
    private boolean purplePurpleGreen = false;
    private AprilTagDetection redBase = null;

    @Override
    public void runOpMode() {
        // Calls a function to initialize the camera
        initAprilTag();

        // Initialize the wheels
        FrontL = hardwareMap.get(DcMotor.class,"FrontL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        BackR = hardwareMap.get(DcMotor.class, "BackR");

        // Sets the wheels directions
        FrontL.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.FORWARD);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the shooter and intake
        Shooter = hardwareMap.get(DcMotor.class,"Shooter");
        intake = hardwareMap.get(DcMotor.class,"intake");

        // Initialize the ball ejector and fidget tech servos
        ballEjector = hardwareMap.get(Servo.class, "ballEjector");
        fidgetTech = hardwareMap.get(Servo.class, "spinIndexer");
        fidgetTech.setPosition(spinPosition[number]);
        waitForStart();
        Shooter.setPower(.9);
        sleep(5000);
        ballEjector.setPosition(.3);
        sleep(100);
        ballEjector.setPosition(0);
        sleep(100);
        number -= 2;
        fidgetTech.setPosition(spinPosition[number]);
        sleep(5000);
        ballEjector.setPosition(.3);
        sleep(100);
        ballEjector.setPosition(0);
        sleep(100);
        number -= 2;
        fidgetTech.setPosition(spinPosition[number]);
        sleep(5000);
        ballEjector.setPosition(.3);
        sleep(100);
        ballEjector.setPosition(0);
        Shooter.setPower(0);
        setWheelPower(-0.5,0,0,1);
        sleep(1000);
        setWheelPower(0,0,0,1);


    }

    private void setWheelPower(double speed, double strafe, double turn, double multiplier) {
        // Sets the motor speeds based on the imputed numbers
        FrontL.setPower((speed - strafe - turn)*multiplier);
        FrontR.setPower((speed + strafe + turn)*multiplier);
        BackL.setPower((speed + strafe - turn)*multiplier);
        BackR.setPower((speed - strafe + turn)*multiplier);
    } // This function controls the driving motors

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    } // This function just sets up the camera for april tags, we put it in a function to get it out of the way

    private void getAprilTagData() {
        blueBase = null;
        redBase = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                if (detection.id == 20) {
                    blueBase = detection;
                }
                if (detection.id == 21) {
                    greenPurplePurple = true;
                }
                if (detection.id == 22) {
                    purpleGreenPurple = true;
                }
                if (detection.id == 23) {
                    purplePurpleGreen = true;
                }
                if (detection.id == 24) {
                    redBase = detection;
                }
            } else {

            }
        }



    }   // end method telemetryAprilTag()
}
