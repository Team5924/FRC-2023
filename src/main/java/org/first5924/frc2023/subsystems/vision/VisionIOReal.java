// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.vision;
import java.io.IOException;

import org.first5924.frc2023.constants.VisionConstants;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonPoseEstimator;



/** Add your docs here. */
public class VisionIOReal implements VisionIO {
    PhotonCamera mCamera;
    PhotonPoseEstimator photonPoseEstimator;


    public VisionIOReal() {
                // Change the name of your camera here to whatever it is in the PhotonVision UI.
                mCamera = new PhotonCamera("PhotonUSBCamera");

                try {
                    // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
                    AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
                    // Create pose estimator
                    photonPoseEstimator =
                            new PhotonPoseEstimator(
                                    fieldLayout, PoseStrategy.MULTI_TAG_PNP, mCamera, VisionConstants.robotToCam);
                    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                } catch (IOException e) {
                    // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
                    // where the tags are.
                    DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
                    photonPoseEstimator = null;
                }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (mCamera.getLatestResult().hasTargets()) {
        inputs.hasTarget =  mCamera.getLatestResult().hasTargets();
        inputs.range = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.kCameraHeightMeters,
            VisionConstants.kTargetHeightMeters,
            VisionConstants.kCameraPitchRadians,
            Units.degreesToRadians(mCamera.getLatestResult().getBestTarget().getPitch()));
        inputs.targetPitch = mCamera.getLatestResult().getBestTarget().getPitch();
        inputs.bestTargetID = mCamera.getLatestResult().getBestTarget().getFiducialId();
        inputs.numberOfTargets = mCamera.getLatestResult().getTargets().size();
        inputs.xAngle= mCamera.getLatestResult().getBestTarget().getYaw();
        }
 }

}
