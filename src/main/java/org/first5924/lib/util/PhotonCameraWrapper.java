// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.lib.util;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class PhotonCameraWrapper {
    private final PhotonCamera mCamera;
    private PhotonPoseEstimator mPhotonPoseEstimator;

    /** Creates a new VisionSubsystem. */
    public PhotonCameraWrapper(String cameraName, Transform3d robotToCam) {
        mCamera = new PhotonCamera(cameraName);
        try {
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            mPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, mCamera, robotToCam);
            mPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            mPhotonPoseEstimator = null;
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        if (mPhotonPoseEstimator == null) {
            return Optional.empty();
        }
        return mPhotonPoseEstimator.update();
    }
}
