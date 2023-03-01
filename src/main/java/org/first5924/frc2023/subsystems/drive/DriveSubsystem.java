// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.drive;

import org.first5924.frc2023.constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  // private final PhotonCameraWrapper mPhotonCameraWrapper = new PhotonCameraWrapper(VisionConstants.kCameraName, new Transform3d(VisionConstants.kRobotToCamTranslation, VisionConstants.kRobotToCamRotation));
  private final DifferentialDrivePoseEstimator mPoseEstimator = new DifferentialDrivePoseEstimator(DriveConstants.kKinematics, getRotation2d(), 0, 0, new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive", inputs);

    mPoseEstimator.update(getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

    Logger.getInstance().recordOutput("Pose Estimation", getEstimatedRobotPose());
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

  public Pose2d getEstimatedRobotPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  public void resetPosition(Pose2d pose) {
    io.setPigeonYaw(pose.getRotation().getDegrees());
    io.resetEncoders();
    mPoseEstimator.resetPosition(getRotation2d(), 0, 0, pose);
  }

  public void addVisionMeasurementToPoseEstimator(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    mPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  public void setPercent(double leftPercent, double rightPercent) {
    io.setPercent(leftPercent, rightPercent);
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

    SmartDashboard.putNumber("Left %", leftPercent);
    SmartDashboard.putNumber("Right %", rightPercent);
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
