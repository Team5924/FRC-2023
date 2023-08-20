// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.first5924.frc2023.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPoseEstimation extends CommandBase {

  private final VisionSubsystem mVision;

  double currentX;
  double currentY;
  boolean visionIsWorking;

  /** Creates a new VisionPoseEstimation. */
  public VisionPoseEstimation(VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    mVision = vision;
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentX = 0;
    currentY = 0;
    visionIsWorking = false;
    
  if (mVision != null) {
    if (mVision.getTagID() == 8) {
      currentX = mVision.xDistance() + Units.inchesToMeters(20);
      currentY = mVision.yDistance() + Units.inchesToMeters(42.185);
    }
    if (mVision.getTagID() == 7) {
      currentX = mVision.xDistance() + Units.inchesToMeters(20);
      currentY = mVision.yDistance() + Units.inchesToMeters(108.185);
    }
    if (mVision.getTagID() == 6) {
      currentX = mVision.xDistance() + Units.inchesToMeters(20);
      currentY = mVision.yDistance() + Units.inchesToMeters(174.185);
    }
    visionIsWorking = true;
    

    }
    Rotation2d RobotRotation = new Rotation2d(0);
    Pose2d RobotPose2d = new Pose2d(currentX, currentY, RobotRotation);
    Logger.getInstance().recordOutput("RobotPose2d", RobotPose2d);
    SmartDashboard.putNumber("currentX", currentX);
    SmartDashboard.putNumber("currentY", currentY);
    Logger.getInstance().recordOutput("currentX", currentX);
    Logger.getInstance().recordOutput("currentY", currentY);
 
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
