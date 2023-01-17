// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.lib.util.Conversions;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax mLeftFrontSpark = new CANSparkMax(DriveConstants.kLeftFrontSparkPort, MotorType.kBrushless);
  private final CANSparkMax mRightFrontSpark = new CANSparkMax(DriveConstants.kRightFrontSparkPort, MotorType.kBrushless);
  private final CANSparkMax mLeftBackSpark = new CANSparkMax(DriveConstants.kLeftBackSparkPort, MotorType.kBrushless);
  private final CANSparkMax mRightBackSpark = new CANSparkMax(DriveConstants.kRightBackSparkPort, MotorType.kBrushless);

  private final WPI_CANCoder mLeftCANCoder = new WPI_CANCoder(DriveConstants.kLeftCANCoderPort);
  private final WPI_CANCoder mRightCANCoder = new WPI_CANCoder(DriveConstants.kRightCANCoderPort);

  private final WPI_Pigeon2 mPigeon2 = new WPI_Pigeon2(RobotConstants.kPigeon2Port);

  private final DifferentialDriveOdometry mOdometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configDriveSpark(mLeftFrontSpark);
    configDriveSpark(mRightFrontSpark);
    configDriveSpark(mLeftBackSpark);
    configDriveSpark(mRightBackSpark);

    mLeftBackSpark.follow(mLeftFrontSpark);
    mRightBackSpark.follow(mRightFrontSpark);

    mRightFrontSpark.setInverted(true);

    mOdometry = new DifferentialDriveOdometry(mPigeon2.getRotation2d(), 0, 0);
  }

  @Override
  public void periodic() {
    mOdometry.update(mPigeon2.getRotation2d(), getLeftCANCoderPositionMeters(), getRightCANCoderPositionMeters());
  }

  public void configDriveSpark(CANSparkMax driveSpark) {
    driveSpark.enableVoltageCompensation(RobotConstants.kNominalVoltage);
    driveSpark.setIdleMode(IdleMode.kBrake);
    driveSpark.setSmartCurrentLimit(42);
  }

  public double getLeftCANCoderPositionMeters() {
    return Conversions.degreesToMeters(mLeftCANCoder.getPosition(), DriveConstants.kWheelCircumferenceMeters);
  }

  public double getRightCANCoderPositionMeters() {
    return Conversions.degreesToMeters(mRightCANCoder.getPosition(), DriveConstants.kWheelCircumferenceMeters);
  }

  public double getLeftCANCoderVelocityMetersPerSecond() {
    return Conversions.degreesPerSecondToMPS(mLeftCANCoder.getVelocity(), DriveConstants.kWheelCircumferenceMeters);
  }

  public double getRightCANCoderVelocityMetersPerSecond() {
    return Conversions.degreesPerSecondToMPS(mRightCANCoder.getVelocity(), DriveConstants.kWheelCircumferenceMeters);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      Conversions.degreesPerSecondToMPS(getLeftCANCoderVelocityMetersPerSecond(), DriveConstants.kWheelCircumferenceMeters),
      Conversions.degreesPerSecondToMPS(getRightCANCoderVelocityMetersPerSecond(), DriveConstants.kWheelCircumferenceMeters)
    );
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public void resetPosition(Pose2d pose) {
    mOdometry.resetPosition(mPigeon2.getRotation2d(), getLeftCANCoderPositionMeters(), getRightCANCoderPositionMeters(), pose);
  }

  public void voltageDrive(double leftVolts, double rightVolts) {
    mLeftFrontSpark.setVoltage(leftVolts);
    mRightFrontSpark.setVoltage(rightVolts);
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

    mLeftFrontSpark.set(leftPercent);
    mRightFrontSpark.set(rightPercent);
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

    mLeftFrontSpark.set(leftPercent);
    mRightFrontSpark.set(rightPercent);
  }
}