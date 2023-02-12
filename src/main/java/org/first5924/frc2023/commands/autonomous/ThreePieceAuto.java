// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;

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
  private final Trajectory mThreeBallA;
  private final Trajectory mThreeBallB;
  private final Trajectory mThreeBallC;
  private final Trajectory mThreeBallD;

  /** Creates a new DriveOneMeter. */
  public ThreePieceAuto(DriveSubsystem drive, Alliance alliance) {
    mThreeBallA = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Three Piece A", 3.5, 3), alliance);
    mThreeBallB = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Three Piece B", 3.5, 3, true), alliance);
    mThreeBallC = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Three Piece C", 3.5, 3), alliance);
    mThreeBallD = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("Three Piece D", 3.5, 3, true), alliance);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        drive.resetPosition(mThreeBallA.getInitialPose());
      }),
      new RamseteCommand(
        mThreeBallA,
        drive::getEstimatedRobotPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        drive::driveVoltage,
        drive
      ),
      new InstantCommand(() -> {
        drive.driveVoltage(0, 0);
      }),
      new RamseteCommand(
        mThreeBallB,
        drive::getEstimatedRobotPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        drive::driveVoltage,
        drive
      ),
      new InstantCommand(() -> {
        drive.driveVoltage(0, 0);
      }),
      new RamseteCommand(
        mThreeBallC,
        drive::getEstimatedRobotPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        drive::driveVoltage,
        drive
      ),
      new InstantCommand(() -> {
        drive.driveVoltage(0, 0);
      }),
      new RamseteCommand(
        mThreeBallD,
        drive::getEstimatedRobotPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        drive::driveVoltage,
        drive
      ),
      new InstantCommand(() -> {
        drive.driveVoltage(0, 0);
      })
    );
  }
}
