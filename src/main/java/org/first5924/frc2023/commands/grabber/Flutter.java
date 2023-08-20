 //Copyright (c) FIRST and other WPILib contributors.
 //Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;

public class Flutter extends CommandBase {
  private final GrabberSubsystem mGrabber;
  private boolean wait = true;
  private final double timeBetweenSwitch = 100;
  private double switchAt = System.currentTimeMillis() + timeBetweenSwitch;

  /** Creates a new Flutter. */
  public Flutter(GrabberSubsystem grabber) {
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
    if (System.currentTimeMillis() >= switchAt) {
      if (wait) {
        wait = false;
        switchAt = System.currentTimeMillis() + timeBetweenSwitch;
      } else {
        wait = true;
        switchAt = System.currentTimeMillis() + timeBetweenSwitch;
      }
    }
    if (wait) {
      mGrabber.runGrabber(0);
    } else {
      mGrabber.runGrabber(0.5);
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