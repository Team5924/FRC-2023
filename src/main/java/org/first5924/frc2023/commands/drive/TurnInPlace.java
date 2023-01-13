// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive;

import java.util.function.DoubleSupplier;

import org.first5924.frc2023.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnInPlace extends CommandBase {
  private final DriveSubsystem mDrive;
  private final DoubleSupplier mLeftJoystickY;
  private final DoubleSupplier mRightJoystickX;

  /** Creates a new CurvatureDrive. */
  public TurnInPlace(DriveSubsystem drive, DoubleSupplier leftJoystickY, DoubleSupplier rightJoystickX) {
    mDrive = drive;
    mLeftJoystickY = leftJoystickY;
    mRightJoystickX = rightJoystickX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.turnInPlace(mLeftJoystickY.getAsDouble(), mRightJoystickX.getAsDouble());
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
