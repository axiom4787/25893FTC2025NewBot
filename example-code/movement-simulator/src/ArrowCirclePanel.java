// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

public class ArrowCirclePanel extends CirclePanel {
  private final RobotPanel robotPanel;

  public ArrowCirclePanel(RobotPanel robotPanel) {
    this.robotPanel = robotPanel;
  }

  @Override
  public void moveDot(int keyCode) {
    super.moveDot(keyCode);

    double steeringAngle = calculateSteeringAngle();
    double speed = calculateSpeed();
    robotPanel.setSteeringAngle(steeringAngle);
    robotPanel.setMaxSpeed(speed);
  }

  private double calculateSteeringAngle() {
    return ((double) (dotX - circleCenterX) / circleRadius) * 45;
  }

  private double calculateSpeed() {
    return ((double) (circleCenterY - dotY) / circleRadius) * 10;
  }
}
