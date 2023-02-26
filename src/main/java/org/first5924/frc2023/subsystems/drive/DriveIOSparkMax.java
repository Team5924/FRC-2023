// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.drive;

import java.util.Optional;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.constants.VisionConstants;
import org.first5924.lib.util.PhotonCameraWrapper;
import org.first5924.lib.util.SparkMaxFactory;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class DriveIOSparkMax implements DriveIO {
    private final CANSparkMax mLeftFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mRightFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mLeftBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mRightBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);

    private final Encoder mLeftEncoder = new Encoder(DriveConstants.kLeftThroughBoreA, DriveConstants.kLeftThroughBoreB);
    private final Encoder mRightEncoder = new Encoder(DriveConstants.kRightThroughBoreA, DriveConstants.kRightThroughBoreB, true);

    private final WPI_Pigeon2 mPigeon2 = new WPI_Pigeon2(DriveConstants.kPigeon2Port);

    private final PhotonCameraWrapper mFrontPhotonCameraWrapper = new PhotonCameraWrapper("front", VisionConstants.kRobotToFrontCam);
    private final PhotonCameraWrapper mBackPhotonCameraWrapper = new PhotonCameraWrapper("back", VisionConstants.kRobotToBackCam);

    public DriveIOSparkMax() {
        mLeftBackSpark.follow(mLeftFrontSpark);
        mRightBackSpark.follow(mRightFrontSpark);

        mLeftFrontSpark.setInverted(true);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionMeters = mLeftEncoder.getDistance() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.rightPositionMeters = mRightEncoder.getDistance() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.leftVelocityMetersPerSec = mLeftEncoder.getRate() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.rightVelocityMetersPerSec = mRightEncoder.getRate() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.pigeonRotationDegrees = mPigeon2.getRotation2d().getDegrees();

        Optional<EstimatedRobotPose> frontOptionalEstimatedRobotPose = mFrontPhotonCameraWrapper.getEstimatedRobotPose();
        Optional<EstimatedRobotPose> backOptionalEstimatedRobotPose = mBackPhotonCameraWrapper.getEstimatedRobotPose();
        if (frontOptionalEstimatedRobotPose.isPresent()) {
            EstimatedRobotPose frontEstimatedRobotPose = frontOptionalEstimatedRobotPose.get();
            inputs.frontEstimatedRobotPoseTranslationX = frontEstimatedRobotPose.estimatedPose.getTranslation().getX();
            inputs.frontEstimatedRobotPoseTranslationY = frontEstimatedRobotPose.estimatedPose.getTranslation().getY();
            inputs.frontEstimatedRobotPoseRotationX = frontEstimatedRobotPose.estimatedPose.getRotation().getX();
            inputs.frontEstimatedRobotPoseRotationY = frontEstimatedRobotPose.estimatedPose.getRotation().getY();
            inputs.frontEstimatedRobotPoseTimestampSeconds = frontEstimatedRobotPose.timestampSeconds;
            int numTargetsUsed = frontEstimatedRobotPose.targetsUsed.size();
            inputs.frontCameraToTargetsTranslationX = new double[numTargetsUsed];
            inputs.frontCameraToTargetsTranslationY = new double[numTargetsUsed];
            inputs.frontCameraToTargetsTranslationZ = new double[numTargetsUsed];
            for (int i = 0; i < numTargetsUsed; i++) {
                inputs.frontCameraToTargetsTranslationX[i] = frontEstimatedRobotPose.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getX();
                inputs.frontCameraToTargetsTranslationY[i] = frontEstimatedRobotPose.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getY();
                inputs.frontCameraToTargetsTranslationZ[i] = frontEstimatedRobotPose.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getZ();
            }
        } else {
            inputs.frontEstimatedRobotPoseTranslationX = Double.NaN;
            inputs.frontEstimatedRobotPoseTranslationY = Double.NaN;
            inputs.frontEstimatedRobotPoseRotationX = Double.NaN;
            inputs.frontEstimatedRobotPoseRotationY = Double.NaN;
            inputs.frontEstimatedRobotPoseTimestampSeconds = Double.NaN;
            inputs.frontCameraToTargetsTranslationX = new double[0];
            inputs.frontCameraToTargetsTranslationY = new double[0];
            inputs.frontCameraToTargetsTranslationZ = new double[0];
        }
        if (backOptionalEstimatedRobotPose.isPresent()) {
            EstimatedRobotPose backEstimatedRobotPose = backOptionalEstimatedRobotPose.get();
            inputs.backEstimatedRobotPoseTranslationX = backEstimatedRobotPose.estimatedPose.getTranslation().getX();
            inputs.backEstimatedRobotPoseTranslationY = backEstimatedRobotPose.estimatedPose.getTranslation().getY();
            inputs.backEstimatedRobotPoseRotationX = backEstimatedRobotPose.estimatedPose.getRotation().getX();
            inputs.backEstimatedRobotPoseRotationY = backEstimatedRobotPose.estimatedPose.getRotation().getY();
            inputs.backEstimatedRobotPoseTimestampSeconds = backEstimatedRobotPose.timestampSeconds;
            int numTargetsUsed = backEstimatedRobotPose.targetsUsed.size();
            inputs.backCameraToTargetsTranslationX = new double[numTargetsUsed];
            inputs.backCameraToTargetsTranslationY = new double[numTargetsUsed];
            inputs.backCameraToTargetsTranslationZ = new double[numTargetsUsed];
            for (int i = 0; i < numTargetsUsed; i++) {
                inputs.backCameraToTargetsTranslationX[i] = backEstimatedRobotPose.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getX();
                inputs.backCameraToTargetsTranslationY[i] = backEstimatedRobotPose.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getY();
                inputs.backCameraToTargetsTranslationZ[i] = backEstimatedRobotPose.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getZ();
            }
        } else {
            inputs.backEstimatedRobotPoseTranslationX = Double.NaN;
            inputs.backEstimatedRobotPoseTranslationY = Double.NaN;
            inputs.backEstimatedRobotPoseRotationX = Double.NaN;
            inputs.backEstimatedRobotPoseRotationY = Double.NaN;
            inputs.backEstimatedRobotPoseTimestampSeconds = Double.NaN;
            inputs.backCameraToTargetsTranslationX = new double[0];
            inputs.backCameraToTargetsTranslationY = new double[0];
            inputs.backCameraToTargetsTranslationZ = new double[0];
        }
    }

    @Override
    public void setPigeonYaw(double yaw) {
        mPigeon2.setYaw(yaw);
    }

    @Override
    public void resetEncoders() {
        mLeftEncoder.reset();
        mRightEncoder.reset();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        mLeftFrontSpark.setVoltage(leftVolts);
        mRightFrontSpark.setVoltage(rightVolts);
    }

    @Override
    public void setPercent(double leftPercent, double rightPercent) {
        mLeftFrontSpark.set(leftPercent);
        mRightFrontSpark.set(rightPercent);
    }
}
