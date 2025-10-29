// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

public interface WheelControlCallback {
  void onSteeringChanged(double steeringAngle); // Adjust steering

  void onSpeedChanged(double speed); // Adjust speed
}
