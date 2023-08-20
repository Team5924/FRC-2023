// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Vision", inputs);
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public double distance () {
    double distance = Math.sqrt(inputs.xDistance * inputs.xDistance + inputs.yDistance * inputs.yDistance);
    return distance;
  }

  public double getTagID () {
    return inputs.bestTargetID;
  }

  public double xAngle() {
    return inputs.xAngle;
  }

  public double targetPitch() {
    return inputs.yAngle;
  }

  public double xDistance() {
    double xDistance= inputs.xDistance;
    return xDistance;
  }

  public double yDistance() {
    double yDistance= inputs.yDistance;
    return yDistance;
  }

  public double zDistance() {
    double zDistance= inputs.zDistance;
    return zDistance;
  }
 

}