// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.drive.tank;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import org.first5924.frc2023.subsystems.drive.DriveSubsystem;

public class TankDrive extends CommandBase {
  private final DriveSubsystem mDrive;
  private final DoubleSupplier mLeftJoystickY;
  private final DoubleSupplier mRightJoystickY;
  /** Creates a new TankDrive. */
  public TankDrive(DriveSubsystem drive, DoubleSupplier leftJoystickY, DoubleSupplier rightJoystickY) {
    mDrive = drive;
    mLeftJoystickY = leftJoystickY;
    mRightJoystickY = rightJoystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.tankDrive(mLeftJoystickY.getAsDouble(), mRightJoystickY.getAsDouble());
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
