// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

public class KeyListenerHandler extends KeyAdapter {
  private final WASDCirclePanel wasdPanel;
  private final ArrowCirclePanel arrowPanel;
  private final RobotPanel robotPanel; // Reference to RobotPanel for rotation

  public KeyListenerHandler(
      WASDCirclePanel wasdPanel, ArrowCirclePanel arrowPanel, RobotPanel robotPanel) {
    this.wasdPanel = wasdPanel;
    this.arrowPanel = arrowPanel;
    this.robotPanel = robotPanel; // Initialize robot panel
  }

  @Override
  public void keyPressed(KeyEvent e) {
    int keyCode = e.getKeyCode();

    switch (keyCode) {
        // WASD keys to move the WASD circle
      case KeyEvent.VK_W, KeyEvent.VK_A, KeyEvent.VK_S, KeyEvent.VK_D -> wasdPanel.moveDot(keyCode);

        // Arrow keys to move the arrow circle
      case KeyEvent.VK_UP, KeyEvent.VK_DOWN, KeyEvent.VK_LEFT, KeyEvent.VK_RIGHT ->
          arrowPanel.moveDot(keyCode);

        // Rotate robot counterclockwise (key 1) and clockwise (key 2)
      case KeyEvent.VK_1 -> robotPanel.rotateRobot(-5); // Rotate counterclockwise
      case KeyEvent.VK_2 -> robotPanel.rotateRobot(5); // Rotate clockwise

      default -> System.out.println("Unhandled key: " + KeyEvent.getKeyText(keyCode));
    }
  }
}
