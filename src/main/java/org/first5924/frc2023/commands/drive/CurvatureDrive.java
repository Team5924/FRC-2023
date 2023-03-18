// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive;

import java.util.function.DoubleSupplier;

import org.first5924.frc2023.constants.OIConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.lib.util.JoystickToOutput;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CurvatureDrive extends CommandBase {
  private final DriveSubsystem mDrive;
  private final DoubleSupplier mLeftJoystickY;
  private final DoubleSupplier mRightJoystickX;
  private final double mMultiplier;

  /** Creates a new CurvatureDrive. */
  public CurvatureDrive(DriveSubsystem drive, DoubleSupplier leftJoystickY, DoubleSupplier rightJoystickX) {
    mDrive = drive;
    mLeftJoystickY = leftJoystickY;
    mRightJoystickX = rightJoystickX;
    mMultiplier = 1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  /** Creates a new CurvatureDrive. */
  public CurvatureDrive(DriveSubsystem drive, DoubleSupplier leftJoystickY, DoubleSupplier rightJoystickX, double multiplier) {
    mDrive = drive;
    mLeftJoystickY = leftJoystickY;
    mRightJoystickX = rightJoystickX;
    mMultiplier = multiplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.curvatureDrive(JoystickToOutput.calculateSquared(-mLeftJoystickY.getAsDouble(), OIConstants.kDriverJoystickDeadband) * mMultiplier, JoystickToOutput.calculateSquared(mRightJoystickX.getAsDouble(), OIConstants.kDriverJoystickDeadband) * 0.8);
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
