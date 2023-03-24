// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.pivot;

import org.first5924.frc2023.constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final PIDController mPID = new PIDController(0.5, 0, 0);


  public PivotSubsystem(PivotIO io) {
    this.io = io;
    setEncoderFromPivotDegrees(PivotConstants.kStartingDegrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Pivot", inputs);
  }

  public double getPivotPositionDegrees() {
    return inputs.pivotPositionDegrees;
  }

  public double getPivotVelocityDegreesPerSecond() {
    return inputs.pivotVelocityDegreesPerSecond;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public void setPosition(double position) {
    io.setVoltage(MathUtil.clamp(mPID.calculate(getPivotPositionDegrees(), position), -4.5, 4.5));
  }

  public void setEncoderFromPivotDegrees(double pivotDegrees) {
    io.setEncoderPosition(pivotDegrees / 360 * PivotConstants.kGearRatio);
  }
}
