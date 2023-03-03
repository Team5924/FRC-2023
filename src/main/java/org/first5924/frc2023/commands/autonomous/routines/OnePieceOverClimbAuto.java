// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.drive.AutoEngageChargeStation;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceOverClimbAuto extends SequentialCommandGroup {
  /** Creates a new DriveOneMeter. */
  public OnePieceOverClimbAuto(DriveSubsystem drive, Alliance alliance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(4),
        new InstantCommand(() -> {
          drive.setPercent(-0.3, -0.3);
        })
      ),
      new InstantCommand(() -> {
        drive.setPercent(0, 0);
      }),
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new InstantCommand(() -> {
          drive.setPercent(0.3, 0.3);
        })
      ),
      new AutoEngageChargeStation(drive, false)
    );
  }
}
