// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.drive.AutoEngageChargeStation;
import org.first5924.frc2023.commands.grabber.Release;
import org.first5924.frc2023.commands.pivot.AutoSetPivot;
import org.first5924.frc2023.constants.AutoConstants;
import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.constants.TelescopeConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;
import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand; 
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceOverClimbAuto extends SequentialCommandGroup {
  /** Creates a new DriveOneMeter. */
  public OnePieceOverClimbAuto(DriveSubsystem drive, PivotSubsystem pivot, GrabberSubsystem grabber, TelescopeSubsystem telescope) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        pivot.setEncoderFromPivotDegrees(PivotConstants.kStartingDegrees);
        telescope.setEncoderFromTelescopeExtensionInches(TelescopeConstants.kStartingExtensionInches);
      }),
      new AutoSetPivot(pivot, 53),
      new ParallelDeadlineGroup(
        new WaitCommand(0.55),
        new Release(grabber)
      ),
      new ParallelDeadlineGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(3.25),
          new InstantCommand(() -> {
            drive.setPercent(-AutoConstants.kChargeStationDriveSpeed, -AutoConstants.kChargeStationDriveSpeed);
          })
        ),
        new AutoSetPivot(pivot, PivotConstants.kStartingDegrees)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(3.9),
        new InstantCommand(() -> {
          drive.setPercent(-AutoConstants.kChargeStationDescentSpeed, -AutoConstants.kChargeStationDescentSpeed);
        })
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25),
        new InstantCommand(() -> {
          drive.setPercent(0, 0);
        })
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(2.25),
        new InstantCommand(() -> {
          drive.setPercent(AutoConstants.kChargeStationDriveSpeed, AutoConstants.kChargeStationDriveSpeed);
        })
      ),
      new AutoEngageChargeStation(drive, true)
    );
  }
}
