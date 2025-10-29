// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import java.awt.*;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;
import javax.swing.*;

public abstract class CirclePanel extends JPanel {
  protected int circleRadius = 100;
  protected int circleCenterX, circleCenterY;
  protected int dotX, dotY;

  private final List<Point> referencePoints = new ArrayList<>();

  public CirclePanel() {
    setPreferredSize(new Dimension(300, 300));
    circleCenterX = getPreferredSize().width / 2;
    circleCenterY = getPreferredSize().height / 2;
    dotX = circleCenterX;
    dotY = circleCenterY;

    // Initialize the grid of possible positions inside the circle
    initializeReferencePoints();
  }

  /** Generates a grid of points that are inside the circle's boundary. */
  private void initializeReferencePoints() {
    int numCircles = 5; // Number of concentric circles
    int pointsPerCircle = 12; // Number of points on each circle

    for (int i = 1; i <= numCircles; i++) {
      double radius = (circleRadius * i) / numCircles; // Radius of the current circle
      double angleStep = 2 * Math.PI / pointsPerCircle;

      for (int j = 0; j < pointsPerCircle; j++) {
        double angle = j * angleStep;
        int x = (int) (circleCenterX + radius * Math.cos(angle));
        int y = (int) (circleCenterY + radius * Math.sin(angle));
        referencePoints.add(new Point(x, y));
      }
    }
  }

  public void moveDot(int keyCode) {
    int step = 5;
    int newX = dotX, newY = dotY;

    switch (keyCode) {
      case KeyEvent.VK_W, KeyEvent.VK_UP -> newY -= step;
      case KeyEvent.VK_S, KeyEvent.VK_DOWN -> newY += step;
      case KeyEvent.VK_A, KeyEvent.VK_LEFT -> newX -= step;
      case KeyEvent.VK_D, KeyEvent.VK_RIGHT -> newX += step;
    }

    if (isInsideCircle(newX, newY)) {
      dotX = newX;
      dotY = newY;
      repaint();
    }
  }

  /** Checks if the given (x, y) position is inside the circle's boundary. */
  private boolean isInsideCircle(int x, int y) {
    int dx = x - circleCenterX;
    int dy = y - circleCenterY;
    return (dx * dx + dy * dy) <= (circleRadius * circleRadius);
  }

  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2d = (Graphics2D) g;
    g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

    // Draw the outer circle
    g2d.setColor(Color.BLACK);
    g2d.drawOval(
        circleCenterX - circleRadius,
        circleCenterY - circleRadius,
        circleRadius * 2,
        circleRadius * 2);

    // Draw the center crosshair
    drawCenterCrosshair(g2d);

    // Draw the grid of reference points inside the circle
    drawReferencePoints(g2d);

    // Draw the red dot
    g2d.setColor(Color.RED);
    g2d.fillOval(dotX - 5, dotY - 5, 10, 10);
  }

  /** Draws the center crosshair. */
  private void drawCenterCrosshair(Graphics2D g2d) {
    g2d.setColor(Color.LIGHT_GRAY);
    g2d.drawLine(
        circleCenterX - circleRadius, circleCenterY, circleCenterX + circleRadius, circleCenterY);
    g2d.drawLine(
        circleCenterX, circleCenterY - circleRadius, circleCenterX, circleCenterY + circleRadius);
  }

  /** Draws small black reference points inside the circle. */
  private void drawReferencePoints(Graphics2D g2d) {
    g2d.setColor(new Color(0, 0, 0, 50)); // Slightly transparent black for subtle effect
    for (Point p : referencePoints) {
      g2d.fillOval(p.x - 2, p.y - 2, 4, 4); // Smaller and subtle points
    }
  }
}
