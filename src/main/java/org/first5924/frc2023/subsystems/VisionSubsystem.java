// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.first5924.frc2023.constants.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera mCamera = new PhotonCamera("camera");
  private final Transform3d mRobotToCam = new Transform3d(VisionConstants.kRobotToCamTranslation, VisionConstants.kRobotToCamRotation);
  private AprilTagFieldLayout mAprilTagFieldLayout;
  private final PhotonPoseEstimator mRobotPoseEstimator = new PhotonPoseEstimator(mAprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, mCamera, mRobotToCam);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    try {
      mAprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch(IOException e) {
      mAprilTagFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), 0, 0);
      System.out.println("Couldn't load AprilTag field layout!");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d prevEstimatedPose) {
    mRobotPoseEstimator.setReferencePose(prevEstimatedPose);
    return mRobotPoseEstimator.update();
  }
}
