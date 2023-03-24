// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.drive.AutoEngageChargeStation;
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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceAuto extends SequentialCommandGroup {
  private final Trajectory mStartToPieceA;
  private final Trajectory mPieceAToSpaceFromStart;
  private final Trajectory mSpaceFromStartToPieceB;
  private final Trajectory mPieceBToSpaceFromStart;

  /** Creates a new DriveOneMeter. */
  public ThreePieceAuto(DriveSubsystem drive, PivotSubsystem pivot, GrabberSubsystem grabber, TelescopeSubsystem telescope, Alliance alliance) {
    mStartToPieceA = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Start to Piece A", 3.5, 3), alliance);
    mPieceAToSpaceFromStart = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Piece A to Space From Start", 3.5, 3, true), alliance);
    mSpaceFromStartToPieceB = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Space From Start to Piece B", 3.5, 3), alliance);
    mPieceBToSpaceFromStart = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Piece B to Space From Start", 3.5, 3, true), alliance);
    Logger.getInstance().recordOutput("3P1", mStartToPieceA);
    Logger.getInstance().recordOutput("3P2", mPieceAToSpaceFromStart);
    Logger.getInstance().recordOutput("3P3", mSpaceFromStartToPieceB);
    Logger.getInstance().recordOutput("3P4", mPieceBToSpaceFromStart);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        pivot.setEncoderFromPivotDegrees(PivotConstants.kStartingDegrees);
        telescope.setEncoderFromTelescopeExtensionInches(TelescopeConstants.kStartingExtensionInches);
      }),
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
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
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
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new RamseteCommand(
        mSpaceFromStartToPieceB,
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
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new RamseteCommand(
        mPieceBToSpaceFromStart,
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
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new AutoEngageChargeStation(drive, false)
    );
  }
}