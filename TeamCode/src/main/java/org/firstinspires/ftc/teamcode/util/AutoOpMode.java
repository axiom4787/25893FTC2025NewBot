package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public abstract class AutoOpMode extends CommandOpModeWithAlliance {
    public Follower follower;
    public TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public Hood hood;
    public Intake intake;
    public Vision vision;
    public Shooter shooter;
    public Turret turret;

    private boolean hasStarted = false;

    @Override
    public final void initialize() {
        super.reset();

        Hardware.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        hood = new Hood();
        intake = new Intake();
        vision = new Vision();
        shooter = new Shooter();
        turret = new Turret();
    }

    @Override
    public final void run() {
        if (!hasStarted) {
            hasStarted = true;

            Globals.setAlliance(alliance);
            follower.setStartingPose(Globals.Close.START_POSE);
            buildPaths();
            schedule(getAutoSequence());
        }

        super.run();

        follower.update();

        // Relocalize
        vision.update(Math.toDegrees(follower.getHeading()));
        vision.updatePose(follower);

        turret.update(follower);
        hood.update(follower);
        shooter.update(follower);

        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public abstract void buildPaths();

    /**
     * Schedule your auto in this method.
     * ex: schedule(new SequentialCommandGroup(...));
     */
    public abstract void scheduleAutoSequence();
}
