// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.drive;

import java.util.Optional;

import org.first5924.frc2023.constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

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

    Logger.getInstance().recordOutput("Pose Estimator Robot Pose", getPoseEstimatorRobotPose());
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

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(inputs.pigeonRotationDegrees);
  }

  public Pose2d getPoseEstimatorRobotPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  private Optional<Pose2d> getFrontCameraEstimatedPose2d () {
    if (!Double.isNaN(inputs.frontEstimatedRobotPoseTimestampSeconds)) {
      Pose2d estimatedPose2d = new Pose2d(
        new Translation2d(inputs.frontEstimatedRobotPoseTranslationX, inputs.frontEstimatedRobotPoseTranslationY),
        new Rotation2d(inputs.frontEstimatedRobotPoseRotationX, inputs.frontEstimatedRobotPoseRotationY)
      );
      return Optional.of(estimatedPose2d);
    } else {
      return Optional.empty();
    }
  }

  private Optional<Double> getFrontCameraTimestampSeconds() {
    if (!Double.isNaN(inputs.frontEstimatedRobotPoseTimestampSeconds)) {
      return Optional.of(inputs.frontEstimatedRobotPoseTimestampSeconds);
    } else {
      return Optional.empty();
    }
  }

  private Optional<Pose2d> getBackCameraEstimatedPose2d () {
    if (!Double.isNaN(inputs.backEstimatedRobotPoseTimestampSeconds)) {
      Pose2d estimatedPose2d = new Pose2d(
        new Translation2d(inputs.backEstimatedRobotPoseTranslationX, inputs.backEstimatedRobotPoseTranslationY),
        new Rotation2d(inputs.backEstimatedRobotPoseRotationX, inputs.backEstimatedRobotPoseRotationY)
      );
      return Optional.of(estimatedPose2d);
    } else {
      return Optional.empty();
    }
  }

  private Optional<Double> getBackCameraTimestampSeconds() {
    if (!Double.isNaN(inputs.backEstimatedRobotPoseTimestampSeconds)) {
      return Optional.of(inputs.backEstimatedRobotPoseTimestampSeconds);
    } else {
      return Optional.empty();
    }
  }

  public void addVisionMeasurementToPoseEstimator(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    mPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public void resetPosition(Pose2d pose) {
    io.setPigeonYaw(pose.getRotation().getDegrees());
    io.resetEncoders();
    mPoseEstimator.resetPosition(getRotation2d(), 0, 0, pose);
  }

  public void driveVoltage(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
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
