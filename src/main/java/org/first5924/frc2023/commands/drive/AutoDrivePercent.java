// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive;

import org.first5924.frc2023.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrivePercent extends CommandBase {
  private final DriveSubsystem mDrive;
  private final double mLeftPercent;
  private final double mRightPercent;
  private final double mDriveTime;
  private final Timer mTimer = new Timer();

  /** Creates a new AutoDrivePercent. */
  public AutoDrivePercent(DriveSubsystem drive, double leftPercent, double rightPercent, double driveTime) {
    mDrive = drive;
    mLeftPercent = leftPercent;
    mRightPercent = rightPercent;
    mDriveTime = driveTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.setPercent(mLeftPercent, mRightPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.setPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.get() > mDriveTime;
  }
}
