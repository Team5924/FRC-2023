// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.drive.AutoEngageChargeStation;
import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
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
public class TwoPieceClimbAuto extends SequentialCommandGroup {
  private final Trajectory mTwoPieceA;
  private final Trajectory mTwoPieceB;
  private final Trajectory mTwoPieceC;

  /** Creates a new DriveOneMeter. */
  public TwoPieceClimbAuto(DriveSubsystem drive, Alliance alliance) {
    mTwoPieceA = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("3 Piece Climb A", 3.5, 3), alliance);
    mTwoPieceB = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("3 Piece Climb B", 3.5, 3, true), alliance);
    mTwoPieceC = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("3 Piece Climb C", 3.5, 3), alliance);
    Logger.getInstance().recordOutput("Two Piece Climb A", mTwoPieceA);
    Logger.getInstance().recordOutput("Two Piece Climb B", mTwoPieceB);
    Logger.getInstance().recordOutput("Two Piece Climb C", mTwoPieceC);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        drive.resetPosition(mTwoPieceA.getInitialPose());
      }),
      new RamseteCommand(
        mTwoPieceA,
        drive::getEstimatedRobotPose,
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
        mTwoPieceB,
        drive::getEstimatedRobotPose,
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
        mTwoPieceC,
        drive::getEstimatedRobotPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        drive::setVoltage,
        drive
      ),
      new AutoEngageChargeStation(drive)
    );
  }
}
