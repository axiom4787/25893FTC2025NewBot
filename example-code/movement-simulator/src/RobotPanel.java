// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import javax.swing.*;

public class RobotPanel extends JPanel {
  private SwerveModule[] wheels;
  private double[] speeds; // Holds the speed of each wheel
  private double maxSpeed = 0.0; // Default speed is 0 (stationary)
  private double currentSteeringAngle = 0.0; // Store the current steering angle
  private double robotRotationAngle = 0.0; // Store the robot's global rotation

  private JLabel dimensionsLabel; // Label for robot dimensions

  public RobotPanel() {
    setLayout(new BorderLayout());

    // Create and add the dimensions label at the top
    dimensionsLabel = new JLabel(getDimensionsText(), SwingConstants.CENTER);
    dimensionsLabel.setFont(new Font("Arial", Font.PLAIN, 16));
    add(dimensionsLabel, BorderLayout.NORTH);

    // Initialize the 4 swerve modules at the correct corners of the chassis
    wheels = new SwerveModule[4];
    speeds = new double[4];
    wheels[0] = new SwerveModule(100, 100); // Front-left
    wheels[1] = new SwerveModule(300, 100); // Front-right
    wheels[2] = new SwerveModule(100, 400); // Rear-left
    wheels[3] = new SwerveModule(300, 400); // Rear-right
  }

  /** Rotates the entire robot (chassis + wheels) by a given angle. */
  public void rotateRobot(double angleIncrement) {
    robotRotationAngle = (robotRotationAngle + angleIncrement) % 360;
    repaint(); // Update the display after rotation
  }

  /** Returns the robot's dimensions as a formatted string. */
  private String getDimensionsText() {
    return String.format("Robot Dimensions: Width = %.2f inches, Length = %.2f inches", 24.0, 30.0);
  }

  /** Allows updating the max driving speed dynamically (forward or reverse). */
  public void setMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    calculateWheelSpeeds(currentSteeringAngle); // Recalculate with current steering angle
    repaint(); // Update the display
  }

  /** Sets the steering angle and updates the wheel rotations accordingly. */
  public void setSteeringAngle(double angle) {
    this.currentSteeringAngle = angle;

    wheels[0].setRotation(angle); // Front-left
    wheels[1].setRotation(angle); // Front-right
    wheels[2].setRotation(-angle); // Rear-left
    wheels[3].setRotation(-angle); // Rear-right

    calculateWheelSpeeds(angle); // Recalculate speeds
    repaint();
  }

  /** Calculates the speed of each wheel based on the steering angle and current speed. */
  private void calculateWheelSpeeds(double angle) {
    double absSpeed = Math.abs(maxSpeed);

    if (absSpeed == 0 || angle == 0) {
      for (int i = 0; i < speeds.length; i++) {
        speeds[i] = absSpeed;
      }
      return;
    }

    double turnRadius = Math.abs(24.0 / (2 * Math.tan(Math.toRadians(angle))));
    double innerSpeed = absSpeed * (turnRadius - 12) / turnRadius;
    double outerSpeed = absSpeed;

    if (angle > 0) { // Turning right
      speeds[0] = outerSpeed;
      speeds[1] = innerSpeed;
      speeds[2] = outerSpeed;
      speeds[3] = innerSpeed;
    } else { // Turning left
      speeds[0] = innerSpeed;
      speeds[1] = outerSpeed;
      speeds[2] = innerSpeed;
      speeds[3] = outerSpeed;
    }

    for (int i = 0; i < speeds.length; i++) {
      speeds[i] *= Math.signum(maxSpeed);
    }
  }

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2d = (Graphics2D) g;
    g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

    // Save the original transform
    AffineTransform oldTransform = g2d.getTransform();

    // Apply global rotation to the entire robot
    g2d.translate(200, 250); // Move origin to the center of the robot
    g2d.rotate(Math.toRadians(robotRotationAngle));
    g2d.translate(-200, -250); // Move back

    // Draw the robot's chassis (blue rectangle)
    g2d.setColor(Color.BLUE);
    g2d.fillRect(100, 100, 200, 300);

    // Draw each wheel and its speed indicator
    for (int i = 0; i < wheels.length; i++) {
      wheels[i].draw(g2d);
      drawSpeedIndicator(g2d, wheels[i], speeds[i]);
    }

    // Draw the dashed pivot line
    drawPivotLine(g2d);

    // Draw the turn radius label below the robot chassis
    drawTurnRadiusLabel(g2d);

    // Restore the original transform
    g2d.setTransform(oldTransform);
  }

  private void drawTurnRadiusLabel(Graphics2D g2d) {
    double turnRadius;
    String radiusText;

    if (currentSteeringAngle == 0) {
      radiusText = "Turn Radius: Straight";
    } else {
      turnRadius = Math.abs(24.0 / (2 * Math.tan(Math.toRadians(currentSteeringAngle))));
      radiusText = String.format("Turn Radius: %.2f inches", turnRadius);
    }

    g2d.setColor(Color.BLACK);
    g2d.drawString(radiusText, 150, 450);
  }

  private void drawPivotLine(Graphics2D g2d) {
    int centerX = 200;
    int centerY = 250;
    double turnRadius = Math.abs(24.0 / (2 * Math.tan(Math.toRadians(currentSteeringAngle))));
    int direction = currentSteeringAngle > 0 ? 1 : -1;

    double endX = centerX + direction * turnRadius;
    Line2D.Double pivotLine = new Line2D.Double(centerX, centerY, endX, centerY);

    g2d.setColor(Color.BLACK);
    g2d.setStroke(
        new BasicStroke(
            2, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10, new float[] {10, 10}, 0));
    g2d.draw(pivotLine);
    g2d.setStroke(new BasicStroke());
  }

  private void drawSpeedIndicator(Graphics2D g2d, SwerveModule wheel, double speed) {
    AffineTransform old = g2d.getTransform();
    g2d.translate(wheel.getX(), wheel.getY());

    int arrowLength = (int) (Math.abs(speed) * 2);
    g2d.setColor(Color.RED);
    g2d.drawLine(0, 0, arrowLength, 0);
    g2d.drawString(String.format("%.2f", speed), arrowLength + 5, 0);

    g2d.setTransform(old);
  }
}
