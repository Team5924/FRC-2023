// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.telescope;

import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoSetTelescope extends CommandBase {
  private final TelescopeSubsystem mTelescope;
  private final double mExtensionInches;

  /** Creates a new SetPivot. */
  public AutoSetTelescope(TelescopeSubsystem telescope, double extensionInches) {
    mTelescope = telescope;
    mExtensionInches = extensionInches;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTelescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mTelescope.setPosition(mExtensionInches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTelescope.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(mExtensionInches - mTelescope.getTelescopeExtensionInches()) < 0.3 && Math.abs(mTelescope.getTelescopeExtensionInchesPerSecond()) < 0.2) || mTelescope.getOutputCurrent() > 40;
  }
}