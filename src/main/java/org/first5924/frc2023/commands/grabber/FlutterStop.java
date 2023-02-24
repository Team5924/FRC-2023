// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.grabber;

import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FlutterStop extends CommandBase {
  private final GrabberSubsystem mGrabber;

  private final Timer timer = new Timer();
  
  /** Creates a new Flutter. */
  public FlutterStop(GrabberSubsystem grabber) {
    mGrabber = grabber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stops the grabber every 0.5 seconds
    // * Requires testing
    mGrabber.runGrabber(0.75);
    timer.start();
    // time has elapsed is in seconds
    if (timer.hasElapsed(0.25)) {
      mGrabber.runGrabber(0);
      timer.stop();
      timer.reset();
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
