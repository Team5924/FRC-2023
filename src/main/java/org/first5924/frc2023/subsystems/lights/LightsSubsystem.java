// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.lights;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  private final LightsIO io;
  private final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();

  /** Creates a new GrabberSubsystem. */
  public LightsSubsystem(LightsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Lights", inputs);
  }

  public void setColor(int r, int g, int b) {
    io.setColor(r, g, b);
  }

  public void animate(Animation animation) {
    io.animate(animation);
}
}