// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.telescope;

import java.util.function.DoubleSupplier;

import org.first5924.frc2023.constants.OIConstants;
import org.first5924.frc2023.constants.TelescopeConstants;
import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendAndRetractTelescope extends CommandBase {
  private final TelescopeSubsystem mTelescope;
  private final DoubleSupplier mJoystickY;

  /** Creates a new Extend. */
  public ExtendAndRetractTelescope(TelescopeSubsystem telescope, DoubleSupplier joystickY) {
    mTelescope = telescope;
    mJoystickY = joystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTelescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mTelescope.getTelescopeExtensionInches()<= TelescopeConstants.kMaxLength && mTelescope.getTelescopeExtensionInches() >= TelescopeConstants.kMinLength) {
      mTelescope.setPercent(MathUtil.applyDeadband(-mJoystickY.getAsDouble(), OIConstants.kOperatorJoystickDeadband) * TelescopeConstants.kSpeedMultiplier);
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
