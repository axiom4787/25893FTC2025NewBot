package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.VendorProductCalibrationIdentity;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationManager;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.xmlpull.v1.XmlPullParser;

import java.util.ArrayList;
import java.util.List;

// TODO IMPORTANT NOTES: For goalTagID, just have separate teleops one for red alliance one for blue where blue teleop can setGoalTagID(20) and red teleop can setGoalTagID(24)
// TODO We will see whether we want separate auto for either alliance, probably yes its just easier that way and there may be some functionality requiring that.
public class AprilTag {
    private int id;
    private int obeliskId;
    private int goalTagID; // our current alliance goal
    private int cameraScannedId;
    private double bearing;
    private double elevation;
    private double range;
    private final VisionPortal portal;
    private final AprilTagProcessor processor;

    public AprilTag(HardwareMap hardwareMap) {
        AprilTagLibrary library = AprilTagGameDatabase.getCurrentGameTagLibrary();

        XmlPullParser parser = hardwareMap.appContext.getResources().getXml(R.xml.teamwebcamcalibrations);
        List<XmlPullParser> parsers = new ArrayList<>();
        parsers.add(parser);

        CameraCalibrationManager camCalManager = new CameraCalibrationManager(parsers);

        CameraCalibrationIdentity identity = new VendorProductCalibrationIdentity(0x046D,0x0825);

        CameraCalibration camCal = camCalManager.getCalibration(
                null, new Size(640, 480)
        );

        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(camCal.focalLengthY, camCal.focalLengthY, camCal.principalPointX, camCal.principalPointY)
                .build();

        WebcamName webcamname = hardwareMap.get(WebcamName.class, "webcam");
        portal = VisionPortal.easyCreateWithDefaults(webcamname, processor);
    }

    public void toggle(boolean bool) {
        portal.setProcessorEnabled(processor, bool);
    }

    public void scanObeliskTag() {
        id = -1;
        List<AprilTagDetection> detectionList = processor.getDetections();

        for (AprilTagDetection detection : detectionList) {
            if (detection.id >= 21 && detection.id <= 23) {
                obeliskId = detection.id;
            }
        }
    }

    public void scanGoalTag() {
        id = -1;
        bearing = Double.NaN;
        elevation = Double.NaN;
        range = Double.NaN;

        // If camera is facing to the right of the center of the cam (if it needs to move to the left) the bearing is positive.
        List<AprilTagDetection> detectionList = processor.getDetections();
        for (AprilTagDetection detection : detectionList) {
            cameraScannedId = detection.id;
            // goalTagID should be gotten before round/during auto
            if (detection.id == goalTagID && detection.ftcPose != null) {
                id = detection.id;
                break;
            }
        }
    }

    public void setGoalTagID(int allianceTagID) {
        goalTagID = allianceTagID;
    }
    public int getCurrentId() { return cameraScannedId;}
    public int getObeliskId(){
        return obeliskId;
    }
    public double getElevation(){
        return elevation;
    }
    public double getRange(){
        return range;
    }
    public double getBearing(){
        return bearing;
    }
}
