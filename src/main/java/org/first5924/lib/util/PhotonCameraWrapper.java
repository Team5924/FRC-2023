// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package org.first5924.lib.util;
//
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.Optional;
//
//import org.photonvision.EstimatedRobotPose;
//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.PhotonPoseEstimator.PoseStrategy;
//
//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Transform3d;
//
///** Add your docs here. */
//public class PhotonCameraWrapper {
//    private final PhotonCamera mCamera;
//    private final Transform3d mRobotToCam;
//    private AprilTagFieldLayout mAprilTagFieldLayout;
//    private final PhotonPoseEstimator mRobotPoseEstimator;
//
//    /** Creates a new VisionSubsystem. */
//    public PhotonCameraWrapper(String cameraName, Transform3d robotToCam) {
//        mCamera = new PhotonCamera(cameraName);
//        mRobotToCam = robotToCam;
//        mRobotPoseEstimator = new PhotonPoseEstimator(mAprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, mCamera, mRobotToCam);
//        try {
//            mAprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
//        } catch(IOException e) {
//            mAprilTagFieldLayout = new AprilTagFieldLayout(new ArrayList<>(), 0, 0);
//            System.out.println("Couldn't load AprilTag field layout!");
//        }
//    }
//
//    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d prevEstimatedPose) {
//        mRobotPoseEstimator.setReferencePose(prevEstimatedPose);
//        return mRobotPoseEstimator.update();
//    }
//}
//