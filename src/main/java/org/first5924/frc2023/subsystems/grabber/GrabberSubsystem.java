// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  private final GrabberIO io;

  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem(GrabberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runGrabber(double percent) {
    io.setPercent(percent);
  }

  public void stopGrabber() {
    io.setPercent(0);
  }
}
