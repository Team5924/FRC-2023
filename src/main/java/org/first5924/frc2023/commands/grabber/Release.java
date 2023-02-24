// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;

public class Release extends CommandBase {
  public final GrabberSubsystem mGrabber;
  /** Creates a new Release. */
  public Release(GrabberSubsystem grabber) {
    mGrabber = grabber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // -1 spins outward
    mGrabber.runGrabber(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mGrabber.runGrabber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
