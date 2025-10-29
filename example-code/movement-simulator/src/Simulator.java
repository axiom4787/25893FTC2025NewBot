// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import java.awt.*;
import javax.swing.*;

public class Simulator extends JFrame {
  public Simulator() {
    setTitle("Swerve Drive Robot Simulator");
    setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    setLayout(new BorderLayout());

    // Create the panels
    RobotPanel robotPanel = new RobotPanel();
    WASDCirclePanel wasdPanel = new WASDCirclePanel();
    ArrowCirclePanel arrowPanel = new ArrowCirclePanel(robotPanel);

    // Add panels to the layout
    add(robotPanel, BorderLayout.CENTER);
    add(wasdPanel, BorderLayout.WEST);
    add(arrowPanel, BorderLayout.EAST);

    // Set the window size and location
    setSize(1000, 800);
    setLocationRelativeTo(null);

    // Create and attach the input handler with the RobotPanel reference
    KeyListenerHandler inputHandler = new KeyListenerHandler(wasdPanel, arrowPanel, robotPanel);
    addKeyListener(inputHandler);

    // Ensure the frame can receive key events
    setFocusable(true);
    requestFocusInWindow();
  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(
        () -> {
          Simulator simulator = new Simulator();
          simulator.setVisible(true);
        });
  }
}
