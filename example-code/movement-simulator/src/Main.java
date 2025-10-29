// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

public class Main {
  public static void main(String[] args) {
    javax.swing.SwingUtilities.invokeLater(
        () -> {
          new Simulator().setVisible(true);
        });
  }
}
