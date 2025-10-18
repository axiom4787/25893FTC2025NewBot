package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public final class Spindexer extends Subsystem {
	private Gamepad gamepad1, gamepad2;
	private final static double DEGREES_TO_SERVO = 1.0 / 360.0;
	private int selectedSegment = 0;
	private boolean isIntakePosition = true;
	private final static double[] INTAKE_DEGREE_POSITIONS = {0.0, 240.0, 120.0};
	private final static double[] TRANSFER_DEGREE_POSITIONS = {180.0, 60.0, 300.0};

	private Servo spindexer;
	private ButtonMap xButtonMap, yButtonMap, bButtonMap, aButtonMap;

	@Override
	public void init(HardwareMap hardwareMap) {
		spindexer = hardwareMap.get(Servo.class, "spindexer");
	}

	@Override
	public void setGamepads(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;

		xButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.WHILE_PRESSED, ButtonMap.Button.X);
		yButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.WHILE_PRESSED, ButtonMap.Button.Y);
		bButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.WHILE_PRESSED, ButtonMap.Button.B);
		aButtonMap = new ButtonMap(gamepad1, ButtonMap.TriggerType.ON_INITIAL_PRESS, ButtonMap.Button.A);
	}

	@Override
	public void update() {
		xButtonMap.handle(() -> {
			selectedSegment = 0;
		});
		yButtonMap.handle(() -> {
			selectedSegment = 1;
		});
		bButtonMap.handle(() -> {
			selectedSegment = 2;
		});

		aButtonMap.handle(() -> {
			isIntakePosition = !isIntakePosition;
		});

		spindexer.setPosition(
			isIntakePosition ?
				INTAKE_DEGREE_POSITIONS[selectedSegment] * DEGREES_TO_SERVO :
				TRANSFER_DEGREE_POSITIONS[selectedSegment] * DEGREES_TO_SERVO
		);
	}

	@Override
	public String getTelemetryData() {
		return String.format("selectedSegment: %d\nisIntakePosition: %b", selectedSegment, isIntakePosition);
	}
}