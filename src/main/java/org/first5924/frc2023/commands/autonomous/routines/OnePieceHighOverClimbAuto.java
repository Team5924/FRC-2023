// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.drive.AutoDrivePercent;
import org.first5924.frc2023.commands.drive.AutoEngageChargeStation;
import org.first5924.frc2023.commands.grabber.RunGrabber;
import org.first5924.frc2023.commands.pivot.AutoSetPivot;
import org.first5924.frc2023.commands.telescope.AutoSetTelescope;
import org.first5924.frc2023.constants.AutoConstants;
import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.constants.TelescopeConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;
import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceHighOverClimbAuto extends SequentialCommandGroup {
  /** Creates a new DriveOneMeter. */
  public OnePieceHighOverClimbAuto(DriveSubsystem drive, PivotSubsystem pivot, GrabberSubsystem grabber, TelescopeSubsystem telescope) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        pivot.setEncoderFromPivotDegrees(PivotConstants.kStartingDegrees);
        telescope.setEncoderFromTelescopeExtensionInches(TelescopeConstants.kStartingExtensionInches);
      }),
      new ParallelCommandGroup(
        new AutoSetPivot(pivot, PivotConstants.kTopGridCube),
        new AutoSetTelescope(telescope, TelescopeConstants.kTopGridCube)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.55),
        new RunGrabber(grabber, -1)
      ),
      new ParallelDeadlineGroup(
        new AutoDrivePercent(drive, -AutoConstants.kChargeStationDriveSpeed, -AutoConstants.kChargeStationDriveSpeed, 2.35),
        new AutoSetPivot(pivot, PivotConstants.kStartingDegrees)
      ),
      new AutoDrivePercent(drive, -AutoConstants.kChargeStationDescentSpeed, -AutoConstants.kChargeStationDescentSpeed, 3.55),
      new AutoDrivePercent(drive, 0, 0, 0.25),
      new AutoDrivePercent(drive, AutoConstants.kChargeStationDriveSpeed, AutoConstants.kChargeStationDriveSpeed, 1.85),
      new AutoEngageChargeStation(drive, true)
    );
  }
}
