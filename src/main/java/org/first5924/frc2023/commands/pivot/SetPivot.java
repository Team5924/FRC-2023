// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.pivot;
import java.util.function.DoubleSupplier;

import org.first5924.frc2023.constants.OIConstants;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPivot extends CommandBase {
  private final PivotSubsystem mPivot;
  private final DoubleSupplier mJoystickY;
  private final double mPosition;

  /** Creates a new SetPivot. */
  public SetPivot(PivotSubsystem pivot, DoubleSupplier joystickY, double position) {
    mPivot = pivot;
    mJoystickY = joystickY;
    mPosition = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mPivot.setPosition(mPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mPivot.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(mJoystickY.getAsDouble()) > OIConstants.kOperatorJoystickDeadband || mPivot.getOutputCurrent() > 35) {
      return true;
    } else {
      return false;
    } 
  }
}
