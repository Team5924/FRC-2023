// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.pivot;
import java.util.function.DoubleSupplier;

import org.first5924.frc2023.constants.OIConstants;
import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotatePivot extends CommandBase {
  private final PivotSubsystem mPivot;
  private final DoubleSupplier mJoystickY;
  private final Boolean mSoftStop;

  //private final PivotIOSparkMax m_pivot;
  //pr0ivate PivotIO m_percentage;
  /** Creates a new RotatePivot. */
  public RotatePivot(PivotSubsystem pivot, DoubleSupplier joystickY, Boolean softStop) {
    mPivot = pivot;
    mJoystickY = joystickY;
    mSoftStop = softStop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mPivot);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mSoftStop == true) {
      if (mPivot.getPivotPositionDegrees() <= 140 && mPivot.getPivotPositionDegrees() >= -140) {
        mPivot.setPercent(MathUtil.applyDeadband(-mJoystickY.getAsDouble(), OIConstants.kOperatorJoystickDeadband) * PivotConstants.kSpeedMultiplier);
      } else if (Math.signum(mPivot.getPivotPositionDegrees()) == Math.signum(mJoystickY.getAsDouble())) {
        mPivot.setPercent(MathUtil.applyDeadband(-mJoystickY.getAsDouble(), OIConstants.kOperatorJoystickDeadband) * PivotConstants.kSpeedMultiplier);
      } else { 
        mPivot.setPercent(0);
      }
    } else {
      mPivot.setPercent(MathUtil.applyDeadband(-mJoystickY.getAsDouble(), OIConstants.kOperatorJoystickDeadband) * PivotConstants.kSpeedMultiplier);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mPivot.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

    
  }
}
