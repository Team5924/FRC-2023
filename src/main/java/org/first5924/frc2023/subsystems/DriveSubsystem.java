// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.constants.RobotConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax mLeftFrontDriveSpark = new CANSparkMax(DriveConstants.kLeftFrontDriveSparkPort, MotorType.kBrushless);
  private final CANSparkMax mRightFrontDriveSpark = new CANSparkMax(DriveConstants.kRightFrontDriveSparkPort, MotorType.kBrushless);
  private final CANSparkMax mLeftBackDriveSpark = new CANSparkMax(DriveConstants.kLeftBackDriveSparkPort, MotorType.kBrushless);
  private final CANSparkMax mRightBackDriveSpark = new CANSparkMax(DriveConstants.kRightBackDriveSparkPort, MotorType.kBrushless);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configDriveSpark(mLeftFrontDriveSpark);
    configDriveSpark(mRightFrontDriveSpark);
    configDriveSpark(mLeftBackDriveSpark);
    configDriveSpark(mRightBackDriveSpark);

    mLeftBackDriveSpark.follow(mLeftFrontDriveSpark);
    mRightBackDriveSpark.follow(mRightFrontDriveSpark);

    mRightFrontDriveSpark.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configDriveSpark(CANSparkMax driveSpark) {
    driveSpark.enableVoltageCompensation(RobotConstants.kNominalVoltage);
    driveSpark.setIdleMode(IdleMode.kBrake);
    driveSpark.setSmartCurrentLimit(42);
  }

  public void curvatureDrive(double xSpeed, double zRotation) {
    double leftPercent = xSpeed + (Math.abs(xSpeed) * zRotation);
    double rightPercent = xSpeed - (Math.abs(xSpeed) * zRotation);

    // Normalize wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftPercent), Math.abs(rightPercent));
    if (maxMagnitude > 1.0) {
      leftPercent /= maxMagnitude;
      rightPercent /= maxMagnitude;
    }

    mLeftFrontDriveSpark.set(leftPercent);
    mRightFrontDriveSpark.set(rightPercent);
  }

  public void turnInPlace(double xSpeed, double zRotation) {
    double leftPercent = xSpeed + zRotation;
    double rightPercent = xSpeed - zRotation;

    // Normalize wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftPercent), Math.abs(rightPercent));
    if (maxMagnitude > 1.0) {
      leftPercent /= maxMagnitude;
      rightPercent /= maxMagnitude;
    }

    mLeftFrontDriveSpark.set(leftPercent);
    mRightFrontDriveSpark.set(rightPercent);
  }
}
