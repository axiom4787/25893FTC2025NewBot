package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.*;

public class InterstellarBot {
	final Subsystem[] subsystems;

	public InterstellarBot(Subsystem... subsystems) {
		this.subsystems = subsystems;
	}

	public void init(HardwareMap hardwareMap) {
		for (Subsystem subsystem : subsystems) {
			subsystem.init(hardwareMap);
		}
	}

	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		for (Subsystem subsystem : subsystems) {
			subsystem.setGamepads(gamepad1, gamepad2);
		}
	}

	public void update() {
		for (Subsystem subsystem : subsystems) {
			subsystem.update();
		}
	}

	public String getTelemetryData() {
		StringBuilder telemetry = new StringBuilder();
		for (Subsystem subsystem: subsystems) {
			telemetry.append(subsystem.getTelemetryData()).append('\n');
		}
		return telemetry.toString();
	}
}