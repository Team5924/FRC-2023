// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final PIDController mPID = new PIDController(0.1, 0, 0);
  private final DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(getRotation2d(), 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    mOdometry.update(getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

    Logger.getInstance().recordOutput("Odometry", getPoseMeters());
  }

  public double getLeftPositionMeters() {
    return inputs.leftPositionMeters;
  }

  public double getRightPositionMeters() {
    return inputs.rightPositionMeters;
  }

  public double getLeftVelocityMetersPerSecond() {
    return inputs.leftVelocityMetersPerSec;
  }

  public double getRightVelocityMetersPerSecond() {
    return inputs.rightVelocityMetersPerSec;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftVelocityMetersPerSecond(),
      getRightVelocityMetersPerSecond()
    );
  }

  public double getYaw() {
    return inputs.pigeonYaw;
  }


  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(inputs.pigeonYaw);
  }

  public double getPitch() {
    return inputs.pigeonPitch;
  }

  public Pose2d getPoseMeters() {
    return mOdometry.getPoseMeters();
  }

  public void resetPosition(Pose2d pose) {
    io.resetEncoders();
    io.setPigeonYaw(pose.getRotation().getDegrees());
    mOdometry.resetPosition(pose.getRotation(), 0, 0, pose);
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  public void setPercent(double leftPercent, double rightPercent) {
    io.setPercent(leftPercent, rightPercent);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    double leftPercent = xSpeed + zRotation;
    double rightPercent = xSpeed - zRotation;

    double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
    double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));

    if (greaterInput == 0.0) {
      io.setPercent(0.0, 0.0);
    } else {
      double saturatedInput = (greaterInput + lesserInput) / greaterInput;
      leftPercent /= saturatedInput;
      rightPercent /= saturatedInput;
      io.setPercent(leftPercent, rightPercent);
    }
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

    io.setPercent(leftPercent, rightPercent);
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

    io.setPercent(leftPercent, rightPercent);
  }
}
