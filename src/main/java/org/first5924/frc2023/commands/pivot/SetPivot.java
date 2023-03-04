// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.pivot;
import java.util.function.DoubleSupplier;

import org.first5924.frc2023.subsystems.pivot.PivotIOSparkMax;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;
import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPivot extends CommandBase {
  private final PivotSubsystem mPivot;
  private final DoubleSupplier mJoystickY;

  /** Creates a new SetPivot. */
  public SetPivot(PivotSubsystem pivot, DoubleSupplier joystickY) {
    mPivot = pivot;
    mJoystickY = joystickY;



    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mPivot.setPIDPosition(4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mPivot.setMotorPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mJoystickY.getAsDouble() > 0.05) {
      return true;
    } 
    else if (mJoystickY.getAsDouble() < -0.05) {
      return true;
    }
    else{
      return false;
    }
  }
}
