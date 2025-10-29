// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import java.awt.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class ControlsPanel extends JPanel {
  private JSlider rotationSlider; // Horizontal slider for steering angle
  private JSlider speedSlider; // Vertical slider for driving speed
  private RobotPanel robotPanel;
  private int currentRotation = 0; // Steering angle (horizontal slider)
  private int currentSpeed = 0; // Driving speed (vertical slider, default 0)

  public ControlsPanel(RobotPanel robotPanel) {
    this.robotPanel = robotPanel;
    setLayout(new BorderLayout());

    // Create the horizontal rotation slider for steering angle
    rotationSlider = new JSlider(JSlider.HORIZONTAL, -10, 10, 0);
    rotationSlider.setMajorTickSpacing(1);
    rotationSlider.setPaintTicks(true);
    rotationSlider.setPaintLabels(true);

    // Listen for horizontal slider changes
    rotationSlider.addChangeListener(
        new ChangeListener() {
          @Override
          public void stateChanged(ChangeEvent e) {
            currentRotation = rotationSlider.getValue();
            robotPanel.setSteeringAngle(currentRotation * 10); // Angle in degrees
          }
        });

    // Create the vertical speed slider for forward and reverse
    speedSlider = new JSlider(JSlider.VERTICAL, -10, 10, 0); // Range: -10 to 10, default 0
    speedSlider.setMajorTickSpacing(1);
    speedSlider.setPaintTicks(true);
    speedSlider.setPaintLabels(true);

    // Listen for vertical slider changes
    speedSlider.addChangeListener(
        new ChangeListener() {
          @Override
          public void stateChanged(ChangeEvent e) {
            currentSpeed = speedSlider.getValue();
            robotPanel.setMaxSpeed(currentSpeed); // Update max speed in RobotPanel
          }
        });

    // Panel for the sliders
    JPanel slidersPanel = new JPanel(new BorderLayout());
    slidersPanel.add(rotationSlider, BorderLayout.SOUTH); // Horizontal at the bottom
    slidersPanel.add(speedSlider, BorderLayout.WEST); // Vertical on the left

    add(slidersPanel, BorderLayout.CENTER); // Add sliders to the main control panel

    // Add key listener to capture arrow key input
    setFocusable(true);
    requestFocusInWindow();
    addKeyListener(
        new KeyAdapter() {
          @Override
          public void keyPressed(KeyEvent e) {
            if (e.getKeyCode() == KeyEvent.VK_RIGHT) {
              if (currentRotation < 10) {
                rotationSlider.setValue(currentRotation + 1);
              }
            } else if (e.getKeyCode() == KeyEvent.VK_LEFT) {
              if (currentRotation > -10) {
                rotationSlider.setValue(currentRotation - 1);
              }
            } else if (e.getKeyCode() == KeyEvent.VK_UP) {
              if (currentSpeed < 10) {
                speedSlider.setValue(currentSpeed + 1);
              }
            } else if (e.getKeyCode() == KeyEvent.VK_DOWN) {
              if (currentSpeed > -10) {
                speedSlider.setValue(currentSpeed - 1);
              }
            }
          }
        });
  }
}
