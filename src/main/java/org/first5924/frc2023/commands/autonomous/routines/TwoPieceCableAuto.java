// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.grabber.RunGrabber;
import org.first5924.frc2023.commands.pivot.AutoSetPivot;
import org.first5924.frc2023.commands.telescope.AutoSetTelescope;
import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.constants.TelescopeConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;
import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceCableAuto extends SequentialCommandGroup {
  private final Trajectory mCableStartToPieceA;
  private final Trajectory mPieceAToSpaceFromCableStart;

  /** Creates a new DriveOneMeter. */
  public TwoPieceCableAuto(DriveSubsystem drive, PivotSubsystem pivot, GrabberSubsystem grabber, TelescopeSubsystem telescope, Alliance alliance) {
    mCableStartToPieceA = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Cable Start to Piece A", 1.5, 2, true), alliance);
    mPieceAToSpaceFromCableStart = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Piece A to Space from Cable Start", 1.5, 2), alliance);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        pivot.setEncoderFromPivotDegrees(PivotConstants.kStartingDegrees);
        telescope.setEncoderFromTelescopeExtensionInches(TelescopeConstants.kStartingExtensionInches);
        drive.resetPosition(mCableStartToPieceA.getInitialPose());
      }),
      new ParallelCommandGroup(
        new AutoSetPivot(pivot, PivotConstants.kTopGridCube),
        new AutoSetTelescope(telescope, TelescopeConstants.kTopGridCube)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new RunGrabber(grabber, -1)
      ),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          mCableStartToPieceA,
          drive::getPoseMeters,
          new RamseteController(),
          new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
          DriveConstants.kKinematics,
          drive::getWheelSpeeds,
          new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
          new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
          drive::setVoltage,
          drive
        ),
        new AutoSetPivot(pivot, -PivotConstants.kGroundPickup),
        new AutoSetTelescope(telescope, TelescopeConstants.kGroundPickup),
        new RunGrabber(grabber, 1)
      ),
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          mPieceAToSpaceFromCableStart,
          drive::getPoseMeters,
          new RamseteController(),
          new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
          DriveConstants.kKinematics,
          drive::getWheelSpeeds,
          new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
          new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
          drive::setVoltage,
          drive
        ),
        new AutoSetPivot(pivot, PivotConstants.kMiddleGridCube),
        new AutoSetTelescope(telescope, TelescopeConstants.kMiddleGridCube)
      ),
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new RunGrabber(grabber, -0.2)
      ),
      new ParallelCommandGroup(
        new AutoSetPivot(pivot, -PivotConstants.kGroundPickup),
        new AutoSetTelescope(telescope, TelescopeConstants.kGroundPickup)
      )
    );
  }
}