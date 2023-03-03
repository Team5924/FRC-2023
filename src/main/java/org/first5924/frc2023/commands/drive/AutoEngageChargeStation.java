// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive;

import org.first5924.frc2023.constants.AutoConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoEngageChargeStation extends CommandBase {
  private final DriveSubsystem mDrive;
  private final boolean mIsFromCenter;
  private final Timer mTimer = new Timer();
  private boolean mWaitingForSettle = false;

  /** Creates a new AutoEngageChargeStation. */
  public AutoEngageChargeStation(DriveSubsystem drive, boolean isFromCenter) {
    mDrive = drive;
    mIsFromCenter = isFromCenter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mWaitingForSettle) {
      if (mTimer.get() > 0.5) {
        mWaitingForSettle = false;
      }
    } else {
      if (mIsFromCenter) {
        if (mDrive.getPitch() < AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(-AutoConstants.kChargeStationDriveSpeed, -AutoConstants.kChargeStationDriveSpeed);
        } else if (mDrive.getPitch() > AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(AutoConstants.kChargeStationDriveSpeed, AutoConstants.kChargeStationDriveSpeed);
        } else {
          mDrive.setPercent(0, 0);
          mWaitingForSettle = true;
        }
      } else {
        if (mDrive.getPitch() < AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(AutoConstants.kChargeStationDriveSpeed, AutoConstants.kChargeStationDriveSpeed);
        } else if (mDrive.getPitch() > AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(-AutoConstants.kChargeStationDriveSpeed, -AutoConstants.kChargeStationDriveSpeed);
        } else {
          mDrive.setPercent(0, 0);
          mWaitingForSettle = true;
        }
      }
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
