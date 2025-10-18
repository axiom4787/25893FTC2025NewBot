package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


public final class Intake extends Subsystem {
	private Gamepad gamepad1, gamepad2;
	private DcMotorEx intake;
	private double intakeSpeed = 0;

	@Override
	public void init(HardwareMap hardwareMap) {
		intake = hardwareMap.get(DcMotorEx.class, "intake");
	}

	@Override
	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void update() {
		if (gamepad1.right_trigger > 0.0) {
			intakeSpeed = gamepad1.right_trigger;
		} else {
			intakeSpeed = -gamepad1.left_trigger;
		}

		intake.setPower(intakeSpeed);
	}

	@Override
	public String getTelemetryData() {
		return String.format("Intake Speed: %f", intakeSpeed);
	}
}