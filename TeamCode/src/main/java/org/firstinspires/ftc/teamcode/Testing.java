package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;
@Disabled
@TeleOp(name = "Testing", group = "Concept")
public class Testing extends LinearOpMode {

    private boolean showRadius = true;


    @Override
    public void runOpMode() {


        // Green and purple detectors
        ColorBlobLocatorProcessor greenLocator =
                new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                        .setDrawContours(false)
                        .setCircleFitColor(Color.GREEN)
                        .setBlurSize(5)
                        .build();

        ColorBlobLocatorProcessor purpleLocator =
                new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                        .setDrawContours(false)
                        .setCircleFitColor(Color.MAGENTA)
                        .setBlurSize(5)
                        .build();

        // Get webcam
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Build VisionPortal
        VisionPortal portal =
                new VisionPortal.Builder()
                        .addProcessor(greenLocator)
                        .addProcessor(purpleLocator)
                        .setCameraResolution(new Size(320, 240))
                        .setCamera(webcam)
                        .build();

        // Adjust camera settings (if supported)
//        portal.Camera().setExposureManual(15); // lower= darker, higher = brighter
//        portal.getCamera().setBrightness(50);     // 0-100
//        portal.getCamera().setContrast(70);       // 0-100

        // Start dashboard stream
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(portal, 30);

        telemetry.setMsTransmissionInterval(100);

        waitForStart();

        while (opModeIsActive()) {

            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();

            // ---------------- Blob Filtering ----------------
            // Contour area filter
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 400, 2000, greenBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 400, 2000, purpleBlobs);

            // Circularity filter
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.7, 1.0, greenBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.7, 1.0, purpleBlobs);

            // Optional radius filter (remove very small or large circles)
            greenBlobs.removeIf(b -> b.getCircle().getRadius() < 8 || b.getCircle().getRadius() > 40);
            purpleBlobs.removeIf(b -> b.getCircle().getRadius() < 8 || b.getCircle().getRadius() > 40);

            // Keep only the largest circle per color
            ColorBlobLocatorProcessor.Blob largestGreen = null;
            ColorBlobLocatorProcessor.Blob largestPurple = null;
            double maxRadiusGreen = 0;
            double maxRadiusPurple = 0;

            for (ColorBlobLocatorProcessor.Blob b : greenBlobs) {
                if (b.getCircle().getRadius() > maxRadiusGreen) {
                    maxRadiusGreen = b.getCircle().getRadius();
                    largestGreen = b;
                }
            }

            for (ColorBlobLocatorProcessor.Blob b : purpleBlobs) {
                if (b.getCircle().getRadius() > maxRadiusPurple) {
                    maxRadiusPurple = b.getCircle().getRadius();
                    largestPurple = b;
                }
            }

            greenBlobs.clear();
            purpleBlobs.clear();
            if (largestGreen != null) greenBlobs.add(largestGreen);
            if (largestPurple != null) purpleBlobs.add(largestPurple);

            boolean greenDetected = largestGreen != null;
            boolean purpleDetected = largestPurple != null;

            // Find closest ball to center
            double frameCenterX = 320 / 2.0;
            double frameCenterY = 240 / 2.0;
            Circle closestCircle = null;
            String closestColor = "None";
            double minDistance = Double.MAX_VALUE;

            for (ColorBlobLocatorProcessor.Blob b : greenBlobs) {
                Circle c = b.getCircle();
                double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
                if (dist < minDistance) {
                    minDistance = dist;
                    closestCircle = c;
                    closestColor = "Green";
                }
            }

            for (ColorBlobLocatorProcessor.Blob b : purpleBlobs) {
                Circle c = b.getCircle();
                double dist = Math.hypot(c.getX() - frameCenterX, c.getY() - frameCenterY);
                if (dist < minDistance) {
                    minDistance = dist;
                    closestCircle = c;
                    closestColor = "Purple";
                }
            }

            boolean greenCloser = "Green".equals(closestColor) && greenDetected;
            boolean purpleCloser = "Purple".equals(closestColor) && purpleDetected;

            // ---------------- Phone telemetry ----------------
            telemetry.addData("Green Detected", greenDetected);
            telemetry.addData("Purple Detected", purpleDetected);
            telemetry.addData("Green Balls", greenBlobs.size());
            telemetry.addData("Purple Balls", purpleBlobs.size());
            telemetry.addData("Green Closer", greenCloser);
            telemetry.addData("Purple Closer", purpleCloser);
            telemetry.addData("Closest Color", closestColor);
            telemetry.update();

            // ---------------- Dashboard telemetry ----------------
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Green Detected", greenDetected);
            packet.put("Purple Detected", purpleDetected);
            packet.put("Green Balls", greenBlobs.size());
            packet.put("Purple Balls", purpleBlobs.size());
            packet.put("Green Closer", greenCloser);
            packet.put("Purple Closer", purpleCloser);
            packet.put("Closest Color", closestColor);
            packet.put("Show Radius", showRadius);
            dashboard.sendTelemetryPacket(packet);

            // Toggle radius display using gamepad A
            if (gamepad1.a) showRadius = !showRadius;

            sleep(50);
        }
    }
}
