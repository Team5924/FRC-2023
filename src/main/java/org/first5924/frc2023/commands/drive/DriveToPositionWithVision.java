// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.vision.VisionSubsystem;
import java.lang.Math;
import edu.wpi.first.math.util.Units;

public class DriveToPositionWithVision extends CommandBase {
  // Drive with vision TURNS the robot till it sees it and DRIVES the robot to set distance

  DriveSubsystem drive;
  VisionSubsystem vision;
  

  double driveSpeed;
  double fullTargetAngle, fullTargetDistance;
  double targetAngle, targetDistance;
  double targetPosX, targetPosY;
  double angleError, distanceError;
  double angleTolerance = 2;
  double distanceTolerance = 5;
  double startTime, currTime;
  boolean timeStarted;

  public DriveToPositionWithVision(DriveSubsystem drive, VisionSubsystem vision, double targetPosX, double targetPosY, double driveSpeed) {
    this.drive = drive;
    this.vision = vision;
    this.targetPosX = targetPosX;
    this.targetPosY = targetPosY;
    this.driveSpeed = driveSpeed;
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = false;
      fullTargetAngle = Math.atan(targetPosY*targetPosY/targetPosX*targetPosX);
      fullTargetDistance = Math.sqrt(targetPosX*targetPosX + targetPosY*targetPosY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetAngle = 0;
    targetDistance = 0;
    if (vision.getTagID() == 8) {
      targetAngle = Math.atan((targetPosY+Units.inchesToMeters(42.185))*(targetPosY+Units.inchesToMeters(42.185))/(targetPosX-Units.inchesToMeters(15.8))*(targetPosX-Units.inchesToMeters(15.8)));
      targetDistance = Math.sqrt((targetPosX-Units.inchesToMeters(15.8))*(targetPosX-Units.inchesToMeters(15.8)) + targetPosY+Units.inchesToMeters(42.185)*targetPosY+Units.inchesToMeters(42.185));
    }
    if (vision.getTagID() == 7) {
      targetAngle = Math.atan((targetPosY+Units.inchesToMeters(108.185))*(targetPosY+Units.inchesToMeters(108.185))/(targetPosX-Units.inchesToMeters(15.8))*(targetPosX-Units.inchesToMeters(15.8)));
      targetDistance = Math.sqrt((targetPosX-Units.inchesToMeters(15.8))*(targetPosX-Units.inchesToMeters(15.8)) + targetPosY+Units.inchesToMeters(108.185)*targetPosY+Units.inchesToMeters(108.185));
    }
    if (vision.getTagID() == 6) {
      targetAngle = Math.atan((targetPosY+Units.inchesToMeters(174.185))*(targetPosY+Units.inchesToMeters(174.185))/(targetPosX-Units.inchesToMeters(15.8))*(targetPosX-Units.inchesToMeters(15.8)));
      targetDistance = Math.sqrt((targetPosX-Units.inchesToMeters(15.8))*(targetPosX-Units.inchesToMeters(15.8)) + targetPosY+Units.inchesToMeters(174.185)*targetPosY+Units.inchesToMeters(174.185));
    }
    if (vision.getTagID() == 5) {
      
    }
    if (vision.getTagID() == 4) {

    }
    if (vision.getTagID() == 3) {
      
    }




    angleError = vision.xAngle() - targetAngle;
    distanceError = vision.distance() - targetDistance;
    SmartDashboard.putNumber("intakeVisionAngleError", angleError);
    SmartDashboard.putNumber("intakeVisionDistanceError", distanceError);

    double turnCorrection = angleError * DriveConstants.kTurnP * -1;
    double driveCorrection = distanceError * 0.0254 * DriveConstants.kPDriveVel / 12;
    //drive.arcadeDrive(driveSpeed, driveCorrection + Math.signum(driveCorrection) * driveSpeed);
    System.out.println("turnpow: " + turnCorrection);
    //drive.arcadeDrive(driveSpeed, turnCorrection + Math.signum(turnCorrection) *  DriveConstants.minTurn);   
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
