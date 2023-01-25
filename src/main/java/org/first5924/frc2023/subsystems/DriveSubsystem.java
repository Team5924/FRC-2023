// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems;

import java.util.Optional;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.frc2023.constants.VisionConstants;
import org.first5924.lib.util.PhotonCameraWrapper;
import org.first5924.lib.util.SparkMaxFactory;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax mLeftFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
  private final CANSparkMax mRightFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
  private final CANSparkMax mLeftBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
  private final CANSparkMax mRightBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);

  private final RelativeEncoder mLeftThroughBore = mLeftFrontSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, RobotConstants.kThroughBoreCPR);
  private final RelativeEncoder mRightThroughBore = mRightFrontSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, RobotConstants.kThroughBoreCPR);

  private final WPI_Pigeon2 mPigeon2 = new WPI_Pigeon2(RobotConstants.kPigeon2Port);

  private final PhotonCameraWrapper mPhotonCameraWrapper = new PhotonCameraWrapper(VisionConstants.kCameraName, new Transform3d(VisionConstants.kRobotToCamTranslation, VisionConstants.kRobotToCamRotation));
  private final DifferentialDrivePoseEstimator mPoseEstimator = new DifferentialDrivePoseEstimator(DriveConstants.kKinematics, mPigeon2.getRotation2d(), 0, 0, new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    mLeftBackSpark.follow(mLeftFrontSpark);
    mRightBackSpark.follow(mRightFrontSpark);

    mRightFrontSpark.setInverted(true);
  }

  @Override
  public void periodic() {
    updatePoseEstimator();
    Optional<EstimatedRobotPose> estimatedRobotPose = mPhotonCameraWrapper.getEstimatedRobotPose(getEstimatedRobotPose());
    if (estimatedRobotPose.isPresent()) {
      addVisionMeasurementToPoseEstimator(estimatedRobotPose.get().estimatedPose.toPose2d(), estimatedRobotPose.get().timestampSeconds);
    }
  }

  public double getLeftThroughBorePositionMeters() {
    return mLeftThroughBore.getPosition() * DriveConstants.kWheelCircumferenceMeters;
  }

  public double getRightThroughBorePositionMeters() {
    return mRightThroughBore.getPosition() * DriveConstants.kWheelCircumferenceMeters;
  }

  public double getLeftThroughBoreVelocityMetersPerSecond() {
    return mLeftThroughBore.getVelocity() * DriveConstants.kWheelCircumferenceMeters / 60;
  }

  public double getRightThroughBoreVelocityMetersPerSecond() {
    return mRightThroughBore.getVelocity() * DriveConstants.kWheelCircumferenceMeters / 60;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftThroughBoreVelocityMetersPerSecond(),
      getRightThroughBoreVelocityMetersPerSecond()
    );
  }

  public Pose2d getEstimatedRobotPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  public void resetPosition(Pose2d pose) {
    mPigeon2.setYaw(pose.getRotation().getDegrees());
    mLeftThroughBore.setPosition(0);
    mRightThroughBore.setPosition(0);
    mPoseEstimator.resetPosition(mPigeon2.getRotation2d(), 0, 0, pose);
  }

  public void updatePoseEstimator() {
    mPoseEstimator.update(mPigeon2.getRotation2d(), getLeftThroughBorePositionMeters(), getRightThroughBorePositionMeters());
  }

  public void addVisionMeasurementToPoseEstimator(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    mPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
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
