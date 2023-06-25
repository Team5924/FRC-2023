// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.vision.VisionSubsystem;

public class DriveToPositionWithVision extends CommandBase {
  // Drive with vision TURNS the robot till it sees it and DRIVES the robot to set distance

  DriveSubsystem drive;
  VisionSubsystem vision;
  

  double targetAngle, targetDistance, driveSpeed;
  double angleError, distanceError;
  double angleTolerance = 2;
  double distanceTolerance = 5;
  double startTime, currTime;
  boolean timeStarted;

  public DriveToPositionWithVision(DriveSubsystem drive, VisionSubsystem vision, double targetAngle, double targetDistance, double driveSpeed) {
    this.drive = drive;
    this.vision = vision;
    this.targetDistance = targetDistance;
    this.driveSpeed = driveSpeed;
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angleError = vision.xAngle() - targetAngle;
    distanceError = vision.range() - targetDistance;
    // SmartDashboard.putNumber("intakeVisionAngleError", angleError);
    // SmartDashboard.putNumber("intakeVisionDistanceError", distanceError);

    double turnCorrection = angleError * DriveConstants.kTurnP * -1;
    double driveCorrection = distanceError * 0.0254 * DriveConstants.kPDriveVel / 12;
    //drive.arcadeDrive(driveCorrection + Math.signum(driveCorrection) * driveSpeed,
    System.out.println("turnpow: " + turnCorrection);
    drive.arcadeDrive(driveSpeed, 
      turnCorrection + Math.signum(turnCorrection) *  DriveConstants.minTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!vision.hasTarget()){
      if(!timeStarted){
        startTime = System.currentTimeMillis();
        timeStarted = true;
      }
      currTime = System.currentTimeMillis();
      if(currTime - startTime >= 2000){
        return true;
      }
    }
    return false;
  }
}