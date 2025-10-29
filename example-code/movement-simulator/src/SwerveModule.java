// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import java.awt.*;
import java.awt.geom.AffineTransform;

public class SwerveModule {
  private int x, y;
  private double rotationAngle = 0;

  public SwerveModule(int x, int y) {
    this.x = x;
    this.y = y;
  }

  public void setRotation(double angle) {
    this.rotationAngle = angle;
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }

  public void draw(Graphics2D g2d) {
    AffineTransform old = g2d.getTransform();

    // Translate to the wheel's position
    g2d.translate(x, y);
    g2d.rotate(Math.toRadians(rotationAngle));

    // Draw wheel as a black rectangle
    g2d.setColor(Color.BLACK);
    g2d.fillRect(-10, -20, 20, 40);

    g2d.setTransform(old); // Reset transformation
  }
}
