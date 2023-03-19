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
  private double mStartSettleTimestamp;
  private boolean misWaitingForSettle = false;

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
    if (misWaitingForSettle) {
      if (Timer.getFPGATimestamp() >= mStartSettleTimestamp + 1.15) {
        misWaitingForSettle = false;
      }
    } else {
      if (mIsFromCenter) {
        if (mDrive.getPitch() < -AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(-AutoConstants.kChargeStationBalanceDriveSpeed, -AutoConstants.kChargeStationBalanceDriveSpeed);
        } else if (mDrive.getPitch() > AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(AutoConstants.kChargeStationBalanceDriveSpeed, AutoConstants.kChargeStationBalanceDriveSpeed);
        } else {
          mDrive.setPercent(0, 0);
          misWaitingForSettle = true;
          mStartSettleTimestamp = Timer.getFPGATimestamp();
        }
      } else {
        if (mDrive.getPitch() < -AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(AutoConstants.kChargeStationBalanceDriveSpeed, AutoConstants.kChargeStationBalanceDriveSpeed);
        } else if (mDrive.getPitch() > AutoConstants.kAllowedChargeStationErrorDegrees) {
          mDrive.setPercent(-AutoConstants.kChargeStationBalanceDriveSpeed, -AutoConstants.kChargeStationBalanceDriveSpeed);
        } else {
          mDrive.setPercent(0, 0);
          misWaitingForSettle = true;
          mStartSettleTimestamp = Timer.getFPGATimestamp();
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
