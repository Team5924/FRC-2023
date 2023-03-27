// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.drive.AutoEngageChargeStation;
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
import org.littletonrobotics.junction.Logger;

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
public class TwoPieceClimbAuto extends SequentialCommandGroup {
  private final Trajectory mStartToPieceA;
  private final Trajectory mPieceAToSpaceFromStart;
  private final Trajectory mSpaceFromStartToPieceBAndStraighten;
  private final Trajectory mPieceBStraightenToClimb;

  /** Creates a new DriveOneMeter. */
  public TwoPieceClimbAuto(DriveSubsystem drive, PivotSubsystem pivot, GrabberSubsystem grabber, TelescopeSubsystem telescope, Alliance alliance) {
    mStartToPieceA = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Start to Piece A", 3.5, 3, true), alliance);
    mPieceAToSpaceFromStart = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Piece A to Space from Start", 3.5, 3), alliance);
    mSpaceFromStartToPieceBAndStraighten = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Space from Start to Piece B and Straighten", 3.5, 3, true), alliance);
    mPieceBStraightenToClimb = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Piece B Straighten to Climb", 2, 3), alliance);
    Logger.getInstance().recordOutput("2P1", mStartToPieceA);
    Logger.getInstance().recordOutput("2P2", mPieceAToSpaceFromStart);
    Logger.getInstance().recordOutput("2P3", mSpaceFromStartToPieceBAndStraighten);
    Logger.getInstance().recordOutput("2P4", mPieceBStraightenToClimb);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        pivot.setEncoderFromPivotDegrees(PivotConstants.kStartingDegrees);
        telescope.setEncoderFromTelescopeExtensionInches(TelescopeConstants.kStartingExtensionInches);
        drive.resetPosition(mStartToPieceA.getInitialPose());
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
          mStartToPieceA,
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
        new AutoSetPivot(pivot, -PivotConstants.kGroundPickupCube),
        new AutoSetTelescope(telescope, TelescopeConstants.kGroundPickupCube),
        new ParallelCommandGroup(
          new WaitCommand(1.75),
          new RunGrabber(grabber, 1)
        )
      ),
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          mPieceAToSpaceFromStart,
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
        new AutoSetTelescope(telescope, TelescopeConstants.kMiddleGridCone)
      ),
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new RunGrabber(grabber, -1)
      ),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          mSpaceFromStartToPieceBAndStraighten,
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
        new AutoSetPivot(pivot, -PivotConstants.kGroundPickupCube),
        new AutoSetTelescope(telescope, TelescopeConstants.kGroundPickupCube),
        new ParallelCommandGroup(
          new WaitCommand(2),
          new ParallelDeadlineGroup(
            new WaitCommand(1.2),
            new RunGrabber(grabber, -1))
        )
      ),
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new ParallelDeadlineGroup(
        new RamseteCommand(
          mPieceBStraightenToClimb,
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
        new AutoSetPivot(pivot, -PivotConstants.kStartingDegrees),
        new AutoSetTelescope(telescope, TelescopeConstants.kStartingExtensionInches)
      ),
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new AutoEngageChargeStation(drive, true)
    );
  }
}