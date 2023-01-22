// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems;

import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.lib.util.SparkMaxFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  private final CANSparkMax mLeaderSpark = SparkMaxFactory.createDefaultSparkMax(PivotConstants.kLeaderSparkPort, MotorType.kBrushless);
  private final CANSparkMax mFollowerSpark = SparkMaxFactory.createDefaultSparkMax(PivotConstants.kFollowerSparkPort, MotorType.kBrushless);

  private final RelativeEncoder mEncoder = mLeaderSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, RobotConstants.kThroughBoreCPR);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    mFollowerSpark.follow(mLeaderSpark);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    mLeaderSpark.set(speed);
  }
}
