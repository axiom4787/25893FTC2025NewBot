package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.List;

public abstract class AutoOpMode extends CommandOpModeWithAlliance {
    public Follower follower;
    public TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public Hood hood;
    public Intake intake;
    public Vision vision;
    public Shooter shooter;
    public Turret turret;

    private boolean hasStarted = false;

    private List<LynxModule> allHubs;
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public final void initialize() {
        super.reset();

        Hardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        hood = new Hood();
        intake = new Intake();
        vision = new Vision(telemetry);
        shooter = new Shooter();
        turret = new Turret();

        allHubs = hardwareMap.getAll(LynxModule.class);
        allHubs.forEach(hub -> {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.setConstant(0x8000FF);
        });
    }

    @Override
    public final void run() {
        allHubs.forEach(LynxModule::clearBulkCache);

        if (!hasStarted) {
            hasStarted = true;

            Globals.setAlliance(alliance);
            follower.setStartingPose(Globals.Close.START_POSE);
            buildPaths();
            scheduleAutoSequence();
        }

        super.run();

        follower.update();
        Globals.Zones.updateRobotLocation(follower);

        // Relocalize
//        vision.update();
//        vision.updatePose(follower);

        turret.update(follower);
        hood.update(follower);
        shooter.update(follower);

        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("looptime (ms)", loopTimer.milliseconds());
        panelsTelemetry.update(telemetry);

        loopTimer.reset();
    }

    /**
     * Build your auto paths here.
     * ex: path = follower.pathBuilder()...build();
     */
    public abstract void buildPaths();

    /**
     * Schedule your auto in this method.
     * ex: schedule(new SequentialCommandGroup(...));
     */
    public abstract void scheduleAutoSequence();
}
