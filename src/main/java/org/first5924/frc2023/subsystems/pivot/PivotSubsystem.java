// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final PIDController mPID = new PIDController(0.1, 0, 0);


  public PivotSubsystem(PivotIO io) {
    this.io = io;
  }

  public double getEncoderPosition() {
    return inputs.encoderPosition;
  }

  public void setMotorPercent(double percent) {
    io.setPercent(percent);
  }

  public void setPIDPosition(double position) {
    mPID.setSetpoint(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Pivot", inputs);
  }
}
