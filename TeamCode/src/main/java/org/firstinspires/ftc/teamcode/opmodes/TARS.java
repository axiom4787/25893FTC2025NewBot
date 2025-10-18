package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.core.InterstellarBot;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LeverTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TARS -JetEngine", group = "Robot")
public class TARS extends LinearOpMode {
	final InterstellarBot tars = new InterstellarBot(
			new Drivebase(1.0, 1.0),
			new Intake(),
			new LeverTransfer(0.3, 0.0),
			new Spindexer()
	);

	@Override
	public void runOpMode() {
		// set up subsystems
		tars.init(hardwareMap);

		// set up gamepads
		tars.setGamepads(gamepad1, gamepad2);

		waitForStart();

		if (isStopRequested()) return;

		while (opModeIsActive()) {
			// run subsystems logic
			tars.update();

			// print telemetry
			try {
				telemetry.addLine(tars.getTelemetryData());
			} catch (Exception e) {
				telemetry.addData("telemetry didn't work", e);
			}
			telemetry.addLine("Honesty setting: 90%");
			telemetry.update();
		}
	}
}