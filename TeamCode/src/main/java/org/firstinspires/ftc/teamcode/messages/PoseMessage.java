// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
  public long timestamp;
  public double x;
  public double y;
  public double heading;

  public PoseMessage(Pose2d pose) {
    this.timestamp = System.nanoTime();
    this.x = pose.position.x;
    this.y = pose.position.y;
    this.heading = pose.heading.toDouble();
  }
}
