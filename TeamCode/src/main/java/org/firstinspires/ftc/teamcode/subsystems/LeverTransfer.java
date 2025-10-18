package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class LeverTransfer extends Subsystem {
	private Gamepad gamepad1, gamepad2;
	private Servo leverTransfer;
	private final double leverDownPosition, leverUpPosition;
	private boolean leverTargetIsUpPosition = false;
	private ButtonMap dpadUpButtonMap, dpadDownButtonMap, dpadLeftButtonMap;

    public LeverTransfer(double leverDownPosition, double leverUpPosition) {
		this.leverDownPosition = leverDownPosition;
		this.leverUpPosition = leverUpPosition;
	}

	@Override
	public void init(HardwareMap hardwareMap) {
		leverTransfer = hardwareMap.get(Servo.class, "leverTransfer");
	}

	@Override
	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;

		dpadUpButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.ON_INITIAL_PRESS, ButtonMap.Button.DPAD_UP);
		dpadDownButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.ON_INITIAL_PRESS, ButtonMap.Button.DPAD_DOWN);
		dpadLeftButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.ON_INITIAL_PRESS, ButtonMap.Button.DPAD_LEFT);
	}

	@Override
	public void update() {
		dpadUpButtonMap.handle(() -> {
			leverTargetIsUpPosition = true;
		});

		dpadDownButtonMap.handle(() -> {
			leverTargetIsUpPosition = false;
		});

		dpadLeftButtonMap.handle(() -> {
			leverTargetIsUpPosition = !leverTargetIsUpPosition;
		});

		leverTransfer.setPosition(leverTargetIsUpPosition ? leverUpPosition : leverDownPosition);
	}

	@Override
	public String getTelemetryData() {
		return String.format("Lever Up Position: %f\nLever Is Up: %b", leverUpPosition, leverTargetIsUpPosition);
	}
}