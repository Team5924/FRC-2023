// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.constants.TelescopeConstants;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;
import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NothingAuto extends InstantCommand {
  private final PivotSubsystem mPivot;
  private final TelescopeSubsystem mTelescope;

  public NothingAuto(PivotSubsystem pivot, TelescopeSubsystem telescope) {
    mPivot = pivot;
    mTelescope = telescope;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mPivot, mTelescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mPivot.setEncoderFromPivotDegrees(PivotConstants.kStartingDegrees);
    mTelescope.setEncoderFromTelescopeExtensionInches(TelescopeConstants.kStartingExtensionInches);
  }
}
