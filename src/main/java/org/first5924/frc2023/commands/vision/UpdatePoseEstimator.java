// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.vision;

import java.util.Optional;

import org.first5924.frc2023.subsystems.DriveSubsystem;
import org.first5924.frc2023.subsystems.VisionSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdatePoseEstimator extends CommandBase {
  private final DriveSubsystem mDrive;
  private final VisionSubsystem mVision;

  /** Creates a new UpdatePoseEstimator. */
  public UpdatePoseEstimator(DriveSubsystem drive, VisionSubsystem vision) {
    mDrive = drive;
    mVision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.updatePoseEstimator();
    Optional<Pair<Pose3d, Double>> optionalPoseEstimatorUpdate = mVision.getPoseEstimatorUpdate();
    if (optionalPoseEstimatorUpdate.isPresent()) {
      Pair<Pose3d, Double> poseEstimatorUpdate = optionalPoseEstimatorUpdate.get();
      mDrive.addVisionMeasurementToPoseEstimator(poseEstimatorUpdate.getFirst().toPose2d(), poseEstimatorUpdate.getSecond());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
